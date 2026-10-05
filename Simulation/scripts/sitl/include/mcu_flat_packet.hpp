// mcu_flat_packet.hpp — OTA-Codec der FLATNESS-Variante, ein Frame je Drohne.
//
// Serialisiert Bus_Cmd_flat in EINEN nRF24-Frame (32 B) und zurueck.
// Quantisierung bit-identisch zur MATLAB-Kette link_tx_flat/link_rx_flat
// (scripts/flatness/); Cross-Check: test_link_flat_codec.cpp gegen die
// Golden-CSV aus dump_link_flat_codec_golden.m.
//
// Uebertragen wird der Zustand (p_ref, v_ref) und der Eingang (a_ref) des
// Referenzmodells, dazu Mocap-Pose und Soll-Gier. j_ref, s_ref und die
// Gier-Ableitungen werden NICHT uebertragen und beim Entpacken zu 0 gesetzt.
//
// Festgelegte Entscheidungen (wie mcu_packet.hpp / Kaskade):
//   * little-endian, int16-Quantisierung qi=clamp(round(v/lsb)), lsb=fs/32767,
//     MATLAB round == half-away-from-zero -> std::lround.
//   * smallest-three-Quat mit reserviertem Codewort 0 = "kein Lagebezug".
//   * fs-Werte MUESSEN mit init_link_flat.m uebereinstimmen (dort dokumentiert).
//
// Byte-Layout (32 B):
//   [0]      hdr: bits[3:0]=id (BCD 0..15), bits[5:4]=estop, bit[6]=ack, bit[7]=0
//   [1]      seq
//   [2..7]   mocap_pos 3x int16 LE
//   [8..11]  q_ext     uint32 LE (sm3)
//   [12..17] p_ref     3x int16 LE
//   [18..23] v_ref     3x int16 LE
//   [24..29] a_ref     3x int16 LE
//   [30..31] yaw_ref   int16 LE (nur yaw)
#ifndef MCU_FLAT_PACKET_HPP
#define MCU_FLAT_PACKET_HPP

#include <cstdint>
#include <cmath>
#include "mcu_packet.hpp"   // pkt::detail — LE-Bytes, quantize, sm3 (eine Quelle)

namespace pktf {

// ---- Paketgeometrie ---------------------------------------------------------
constexpr int SIZE = 32;
namespace off {
constexpr int HDR = 0, SEQ = 1;
constexpr int MOC = 2, QE = 8, P = 12, V = 18, A = 24, Y = 30;
}  // namespace off
constexpr uint8_t ID_MASK = 0x0F;

// ---- int16-Skalen (muessen zu init_link_flat.m passen) ----------------------
constexpr double FS_MOC  = 20.0;    // mocap_pos [m]
constexpr double FS_PREF = 20.0;    // p_ref     [m]
constexpr double FS_VREF = 20.0;    // v_ref     [m/s]
constexpr double FS_AREF = 50.0;    // a_ref     [m/s^2]
constexpr double FS_YAW  = 4.0;     // yaw_ref   [rad]

// ---- Bus_Cmd_flat-Spiegel (POD, Feldreihenfolge == setup_buses.m) -----------
struct CmdFlat {
    double mocap_pos[3];
    double q_ext[4];      // scalar-first [w x y z]
    double p_ref[3];
    double v_ref[3];
    double a_ref[3];
    double j_ref[3];
    double s_ref[3];
    double yaw_ref[3];
    uint8_t estop;        // 0/1/2
    bool ack;
};

namespace detail {
using pkt::detail::put_i16;
using pkt::detail::get_i16;
using pkt::detail::put_u32;
using pkt::detail::get_u32;
using pkt::detail::quantize;
using pkt::detail::dequantize;
using pkt::detail::pack_quat;
using pkt::detail::unpack_quat;

inline uint8_t make_hdr(const CmdFlat& c, uint8_t id) {
    return static_cast<uint8_t>((id & ID_MASK) | ((c.estop & 0x03) << 4) |
                                (c.ack ? 0x40 : 0x00));
}
inline void put_vec3(uint8_t* p, const double v[3], double fs) {
    for (int i = 0; i < 3; ++i) put_i16(p + 2*i, quantize(v[i], fs));
}
inline void get_vec3(const uint8_t* p, double v[3], double fs) {
    for (int i = 0; i < 3; ++i) v[i] = dequantize(get_i16(p + 2*i), fs);
}
}  // namespace detail

// --- API ---------------------------------------------------------------------

// Bus_Cmd_flat + id/seq -> OTA-Puffer.
inline void pack(const CmdFlat& c, uint8_t id, uint8_t seq, uint8_t buf[SIZE]) {
    buf[off::HDR] = detail::make_hdr(c, id);
    buf[off::SEQ] = seq;
    detail::put_vec3(buf + off::MOC, c.mocap_pos, FS_MOC);
    detail::put_u32 (buf + off::QE,  detail::pack_quat(c.q_ext));
    detail::put_vec3(buf + off::P,   c.p_ref, FS_PREF);
    detail::put_vec3(buf + off::V,   c.v_ref, FS_VREF);
    detail::put_vec3(buf + off::A,   c.a_ref, FS_AREF);
    detail::put_i16 (buf + off::Y,   detail::quantize(c.yaw_ref[0], FS_YAW));
}

// Header-Zugriff.
inline uint8_t id_of(const uint8_t* buf)   { return buf[off::HDR] & ID_MASK; }
inline bool    id_matches(const uint8_t* buf, uint8_t own_id) { return id_of(buf) == (own_id & ID_MASK); }
inline uint8_t seq_of(const uint8_t* buf)  { return buf[off::SEQ]; }

// OTA-Puffer -> CmdFlat. j_ref, s_ref und die Gier-Ableitungen werden 0.
inline void unpack(const uint8_t buf[SIZE], CmdFlat& c) {
    uint8_t h = buf[off::HDR];
    c.estop = static_cast<uint8_t>((h >> 4) & 0x03);
    c.ack   = ((h >> 6) & 0x01) != 0;
    detail::get_vec3(buf + off::MOC, c.mocap_pos, FS_MOC);
    detail::unpack_quat(detail::get_u32(buf + off::QE), c.q_ext);
    detail::get_vec3(buf + off::P, c.p_ref, FS_PREF);
    detail::get_vec3(buf + off::V, c.v_ref, FS_VREF);
    detail::get_vec3(buf + off::A, c.a_ref, FS_AREF);
    for (int i = 0; i < 3; ++i) { c.j_ref[i] = 0.0; c.s_ref[i] = 0.0; }
    c.yaw_ref[0] = detail::dequantize(detail::get_i16(buf + off::Y), FS_YAW);
    c.yaw_ref[1] = 0.0;
    c.yaw_ref[2] = 0.0;
}

}  // namespace pktf
#endif  // MCU_FLAT_PACKET_HPP
