// gcs_frame.hpp — USB-Frame GS(Simulink) -> Sende-Teensy (Design A).
//
// Vollpraeziser (float32) Bus_Cmd-Frame ueber USB-Serial. Der Sende-Teensy parst
// ihn, haelt seq pro Drohne und quantisiert via pkt::pack (mcu_packet.hpp) in das
// 29-B-OTA-Paket. Dieser Header ist die gemeinsame Quelle fuer beide Enden; die
// Simulink-Seite (Serial Send) repliziert genau dieses Byte-Layout. Cross-Check:
// test_gcs_frame.
//
// Festgelegt: float32, little-endian (Win-x86 GS und Cortex-M7 sind beide LE),
//   fixe Laenge, Sync 0xAA55, CRC-8/SMBus (Poly 0x07, Init 0x00) ueber id+Payload.
//   Die nRF-HW-CRC deckt die Funkstrecke separat ab; dieses CRC schuetzt den USB-Frame.
//
// Byte-Layout (82 B):
//   [0]      0xAA          Sync high
//   [1]      0x55          Sync low
//   [2]      id            Ziel-Drohne (BCD 0..15)
//   [3..78]  19x float32   Bus_Cmd: F_des, q_des[4], q_ref[4], Omega_ref[3],
//                          tau_ref[3], q_ext[4]  (Reihenfolge == setup_buses.m)
//   [79]     estop         0/1/2
//   [80]     ack           0/1
//   [81]     crc8          ueber Bytes [2..80] (id + Payload + estop + ack)
#ifndef GCS_FRAME_HPP
#define GCS_FRAME_HPP

#include <cstdint>
#include <cstring>

namespace gcs {

constexpr uint8_t SYNC0 = 0xAA, SYNC1 = 0x55;
constexpr int SIZE = 82;
namespace off {
constexpr int SYNC = 0, ID = 2, PAY = 3, ESTOP = 79, ACK = 80, CRC = 81;
constexpr int CRC_BEGIN = 2, CRC_LEN = 79;   // [id .. ack]
}  // namespace off

// float32-Spiegel des Bus_Cmd (Reihenfolge == setup_buses.m).
struct GcsCmd {
    float F_des;
    float q_des[4];
    float q_ref[4];
    float Omega_ref[3];
    float tau_ref[3];
    float q_ext[4];
    uint8_t estop;
    uint8_t ack;
};

namespace detail {

inline void put_f32(uint8_t* p, float v) {
    uint32_t u; std::memcpy(&u, &v, 4);           // bit-pattern, dann LE
    p[0] = uint8_t(u); p[1] = uint8_t(u >> 8); p[2] = uint8_t(u >> 16); p[3] = uint8_t(u >> 24);
}
inline float get_f32(const uint8_t* p) {
    uint32_t u = uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
    float v; std::memcpy(&v, &u, 4); return v;
}
// CRC-8/SMBus: Poly 0x07, Init 0x00, kein Reflect, kein XorOut.
inline uint8_t crc8(const uint8_t* d, int n) {
    uint8_t c = 0x00;
    for (int i = 0; i < n; ++i) {
        c ^= d[i];
        for (int b = 0; b < 8; ++b) c = (c & 0x80) ? uint8_t((c << 1) ^ 0x07) : uint8_t(c << 1);
    }
    return c;
}

}  // namespace detail

// GcsCmd + id -> 82-B-Frame.
inline void build(const GcsCmd& c, uint8_t id, uint8_t buf[SIZE]) {
    buf[off::SYNC] = SYNC0; buf[off::SYNC + 1] = SYNC1;
    buf[off::ID] = id;
    uint8_t* p = buf + off::PAY;
    detail::put_f32(p, c.F_des); p += 4;
    for (int i = 0; i < 4; ++i) { detail::put_f32(p, c.q_des[i]); p += 4; }
    for (int i = 0; i < 4; ++i) { detail::put_f32(p, c.q_ref[i]); p += 4; }
    for (int i = 0; i < 3; ++i) { detail::put_f32(p, c.Omega_ref[i]); p += 4; }
    for (int i = 0; i < 3; ++i) { detail::put_f32(p, c.tau_ref[i]); p += 4; }
    for (int i = 0; i < 4; ++i) { detail::put_f32(p, c.q_ext[i]); p += 4; }
    buf[off::ESTOP] = c.estop;
    buf[off::ACK] = c.ack;
    buf[off::CRC] = detail::crc8(buf + off::CRC_BEGIN, off::CRC_LEN);
}

// 82-B-Frame -> GcsCmd + id. Prueft Sync + CRC; false bei Fehler (Bytes bleiben unberuehrt).
inline bool parse(const uint8_t buf[SIZE], GcsCmd& c, uint8_t& id) {
    if (buf[off::SYNC] != SYNC0 || buf[off::SYNC + 1] != SYNC1) return false;
    if (detail::crc8(buf + off::CRC_BEGIN, off::CRC_LEN) != buf[off::CRC]) return false;
    id = buf[off::ID];
    const uint8_t* p = buf + off::PAY;
    c.F_des = detail::get_f32(p); p += 4;
    for (int i = 0; i < 4; ++i) { c.q_des[i] = detail::get_f32(p); p += 4; }
    for (int i = 0; i < 4; ++i) { c.q_ref[i] = detail::get_f32(p); p += 4; }
    for (int i = 0; i < 3; ++i) { c.Omega_ref[i] = detail::get_f32(p); p += 4; }
    for (int i = 0; i < 3; ++i) { c.tau_ref[i] = detail::get_f32(p); p += 4; }
    for (int i = 0; i < 4; ++i) { c.q_ext[i] = detail::get_f32(p); p += 4; }
    c.estop = buf[off::ESTOP];
    c.ack = buf[off::ACK];
    return true;
}

}  // namespace gcs
#endif  // GCS_FRAME_HPP
