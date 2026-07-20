// mcu_packet.hpp — OTA-Codec, gemeinsame Quelle fuer Host und Firmware.
//
// Serialisiert Bus_Cmd in das 29-Byte-OTA-Paket und zurueck. Bit-identisch zur
// MATLAB-Kette link_tx/link_rx (chart_40/chart_50 in link.slx) plus pack_quat_sm3/
// unpack_quat_sm3. Der Host-Test test_link_codec.cpp verifiziert das gegen die
// Golden-CSV aus dump_link_codec_golden.m.
//
// Festgelegte Entscheidungen:
//   * Ziel-HW Teensy 4.1 (Cortex-M7), double behalten.
//   * Quaternionen scalar-first [w x y z].
//   * int16 nur fuer [F_des | Omega_ref(3) | tau_ref(3)], fs/qmax wie init_link.
//   * Multibyte-Felder little-endian (beide Enden ARM-LE, internes Protokoll).
//   * MATLAB round == half-away-from-zero  -> std::lround (nicht nearbyint).
//
// Byte-Layout (29 B):
//   [0]     id
//   [1]     flags:  bits[1:0]=estop (0/1/2), bit[2]=ack, Rest 0
//   [2]     seq
//   [3..4]  F_des        int16  LE
//   [5..8]  q_des        uint32 LE (smallest-three)
//   [9..12] q_ref        uint32 LE
//   [13..16]q_ext        uint32 LE
//   [17..22]Omega_ref    3x int16 LE
//   [23..28]tau_ref      3x int16 LE
#ifndef MCU_PACKET_HPP
#define MCU_PACKET_HPP

#include <cstdint>
#include <cmath>

namespace pkt {

// ---- Paketgeometrie (Test liest die Offsets ebenfalls hierher) --------------
constexpr int SIZE = 29;
namespace off {
constexpr int ID = 0, FLAGS = 1, SEQ = 2, F = 3, QD = 5, QR = 9, QE = 13,
              OM = 17, TAU = 23;
}  // namespace off

// ---- int16-Quantisierung (muss zu init_link.m passen) -----------------------
//   fs-Reihenfolge im 7er-Vektor: [F_des | Omega_ref(3) | tau_ref(3)].
constexpr double QMAX = 32767.0;
constexpr double QMIN = -32768.0;
constexpr double FS_F = 40.0;    // F_des    [N]
constexpr double FS_OM = 10.0;   // Omega_ref[rad/s]
constexpr double FS_TAU = 2.0;   // tau_ref  [N*m]

// ---- Bus_Cmd-Spiegel (POD) --------------------------------------------------
struct Cmd {
    double F_des;
    double q_des[4];      // scalar-first [w x y z]
    double q_ref[4];
    double Omega_ref[3];
    double tau_ref[3];
    double q_ext[4];
    uint8_t estop;        // 0/1/2
    bool ack;
};

// --- intern: LE-Bytezugriff ---
namespace detail {

inline void put_i16(uint8_t* p, int16_t v) {
    uint16_t u = static_cast<uint16_t>(v);            // two's complement
    p[0] = static_cast<uint8_t>(u & 0xFF);
    p[1] = static_cast<uint8_t>((u >> 8) & 0xFF);
}
inline int16_t get_i16(const uint8_t* p) {
    uint16_t u = static_cast<uint16_t>(p[0]) |
                 (static_cast<uint16_t>(p[1]) << 8);
    return static_cast<int16_t>(u);
}
inline void put_u32(uint8_t* p, uint32_t u) {
    p[0] = static_cast<uint8_t>(u & 0xFF);
    p[1] = static_cast<uint8_t>((u >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((u >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((u >> 24) & 0xFF);
}
inline uint32_t get_u32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

// int16-Quantisierung: qi = clamp(round(v/lsb), QMIN, QMAX), lsb = fs/QMAX.
// Rechenreihenfolge identisch zu link_tx.m (erst lsb=fs/qmax, dann v./lsb).
inline int16_t quantize(double v, double fs) {
    double lsb = fs / QMAX;
    double qi = std::lround(v / lsb);                 // half-away-from-zero
    if (qi > QMAX) qi = QMAX;
    if (qi < QMIN) qi = QMIN;
    return static_cast<int16_t>(qi);
}
inline double dequantize(int16_t p, double fs) {
    return static_cast<double>(p) * (fs / QMAX);      // = p .* lsb
}

// ---------------- smallest-three Quaternion-Codec ----------------------------
// Bit-identisch zu pack_quat_sm3.m / unpack_quat_sm3.m.
inline uint32_t pack_quat(const double q_in[4]) {
    double q[4];
    double n = std::sqrt(q_in[0]*q_in[0] + q_in[1]*q_in[1] +
                         q_in[2]*q_in[2] + q_in[3]*q_in[3]);
    if (n < 1e-12) { q[0]=1; q[1]=0; q[2]=0; q[3]=0; }
    else { for (int i=0;i<4;++i) q[i]=q_in[i]/n; }

    int imax = 0; double amax = std::fabs(q[0]);
    for (int i=1;i<4;++i) { double a=std::fabs(q[i]); if (a>amax){amax=a;imax=i;} }

    if (q[imax] < 0.0) { for (int i=0;i<4;++i) q[i]=-q[i]; }  // groesste positiv

    const double SCALE = 511.0 * std::sqrt(2.0);
    uint32_t code = static_cast<uint32_t>(imax) << 30;        // 2-bit Index
    int slot = 20;
    for (int i=0;i<4;++i) {
        if (i != imax) {
            long qi = std::lround(q[i] * SCALE);
            if (qi >  511) qi =  511;
            if (qi < -511) qi = -511;
            uint32_t u = static_cast<uint32_t>(qi + 512);     // Offset-Binary [1..1023]
            code |= (u << slot);
            slot -= 10;
        }
    }
    return code;
}

inline void unpack_quat(uint32_t code, double q_out[4]) {
    int imax = static_cast<int>(code >> 30);                  // 0..3
    double u[3];
    u[0] = static_cast<double>((code >> 20) & 0x3FFu);
    u[1] = static_cast<double>((code >> 10) & 0x3FFu);
    u[2] = static_cast<double>( code        & 0x3FFu);

    const double SCALE = 511.0 * std::sqrt(2.0);
    double c[3];
    for (int k=0;k<3;++k) c[k] = (u[k] - 512.0) / SCALE;

    double q[4]; int k=0; double ssum=0.0;
    for (int i=0;i<4;++i) {
        if (i != imax) { q[i]=c[k]; ssum += c[k]*c[k]; ++k; }
    }
    double rem = 1.0 - ssum;
    q[imax] = std::sqrt(rem > 0.0 ? rem : 0.0);               // groesste, positiv

    double n = std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (n < 1e-12) { q_out[0]=1; q_out[1]=0; q_out[2]=0; q_out[3]=0; }
    else { for (int i=0;i<4;++i) q_out[i]=q[i]/n; }
}

}  // namespace detail

// --- API ---

// Bus_Cmd + id/seq -> 29-Byte-OTA-Puffer.
inline void pack(const Cmd& c, uint8_t id, uint8_t seq, uint8_t buf[SIZE]) {
    buf[off::ID]  = id;
    buf[off::SEQ] = seq;
    buf[off::FLAGS] = static_cast<uint8_t>((c.estop & 0x03) | (c.ack ? 0x04 : 0x00));

    detail::put_i16(buf + off::F, detail::quantize(c.F_des, FS_F));
    detail::put_u32(buf + off::QD, detail::pack_quat(c.q_des));
    detail::put_u32(buf + off::QR, detail::pack_quat(c.q_ref));
    detail::put_u32(buf + off::QE, detail::pack_quat(c.q_ext));
    for (int i=0;i<3;++i) detail::put_i16(buf + off::OM  + 2*i, detail::quantize(c.Omega_ref[i], FS_OM));
    for (int i=0;i<3;++i) detail::put_i16(buf + off::TAU + 2*i, detail::quantize(c.tau_ref[i],   FS_TAU));
}

// 29-Byte-OTA-Puffer -> Bus_Cmd + id/seq.
inline void unpack(const uint8_t buf[SIZE], Cmd& c, uint8_t& id, uint8_t& seq) {
    id  = buf[off::ID];
    seq = buf[off::SEQ];
    uint8_t f = buf[off::FLAGS];
    c.estop = static_cast<uint8_t>(f & 0x03);
    c.ack   = ((f >> 2) & 0x01) != 0;

    c.F_des = detail::dequantize(detail::get_i16(buf + off::F), FS_F);
    detail::unpack_quat(detail::get_u32(buf + off::QD), c.q_des);
    detail::unpack_quat(detail::get_u32(buf + off::QR), c.q_ref);
    detail::unpack_quat(detail::get_u32(buf + off::QE), c.q_ext);
    for (int i=0;i<3;++i) c.Omega_ref[i] = detail::dequantize(detail::get_i16(buf + off::OM  + 2*i), FS_OM);
    for (int i=0;i<3;++i) c.tau_ref[i]   = detail::dequantize(detail::get_i16(buf + off::TAU + 2*i), FS_TAU);
}

// --- Firmware-Komfort (Design A, drone_hal.cpp) ------------------------------
// App-ID-Gate: Broadcast, jede Drohne nimmt nur ihr eigenes id-Byte (buf[0]).
inline bool id_matches(const uint8_t buf[SIZE], uint8_t own_id) {
    return buf[off::ID] == own_id;
}
// unpack ohne id/seq (die HAL braucht nur das Bus_Cmd; id ist per Gate schon geprueft).
inline void unpack(const uint8_t buf[SIZE], Cmd& c) {
    uint8_t id, seq; unpack(buf, c, id, seq);
}

}  // namespace pkt
#endif  // MCU_PACKET_HPP
