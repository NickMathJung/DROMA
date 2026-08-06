// gcs_frame_flat.hpp — USB-Frame GS(Simulink) -> Sende-Teensy, FLATNESS-Variante.
//
// Pendant zu gcs_frame.hpp (Kaskade). Vollpraeziser (float32) Bus_Cmd_flat-Frame
// ueber USB-Serial. Der Sende-Teensy parst ihn und quantisiert via pktf::pack
// (mcu_flat_packet.hpp) in die ZWEI 27-B-OTA-Frames. Dieser Header ist die
// gemeinsame Quelle fuer beide Enden; die Simulink-Seite (Serial Send)
// repliziert genau dieses Byte-Layout. Cross-Check: test_gcs_frame_flat.
//
// Festgelegt wie bei der Kaskade: float32, little-endian, fixe Laenge,
//   Sync 0xAA55, CRC-8/SMBus (Poly 0x07, Init 0x00) ueber id+Payload+estop+ack.
//   Die nRF-HW-CRC deckt die Funkstrecke separat ab; dieses CRC schuetzt USB.
//
// Byte-Layout (106 B):
//   [0]        0xAA          Sync high
//   [1]        0x55          Sync low
//   [2]        id            Ziel-Drohne (BCD 0..15)
//   [3..102]   25x float32   Bus_Cmd_flat: mocap_pos[3], q_ext[4], p_ref[3],
//                            v_ref[3], a_ref[3], j_ref[3], s_ref[3], yaw_ref[3]
//                            (Reihenfolge == setup_buses.m)
//   [103]      estop         0/1/2
//   [104]      ack           0/1
//   [105]      crc8          ueber Bytes [2..104]
#ifndef GCS_FRAME_FLAT_HPP
#define GCS_FRAME_FLAT_HPP

#include <cstdint>
#include <cstring>
#include "gcs_frame.hpp"   // gcs::detail — put_f32/get_f32/crc8 (eine Quelle)

namespace gcsf {

constexpr uint8_t SYNC0 = 0xAA, SYNC1 = 0x55;
constexpr int SIZE = 106;
namespace off {
constexpr int SYNC = 0, ID = 2, PAY = 3, ESTOP = 103, ACK = 104, CRC = 105;
constexpr int CRC_BEGIN = 2, CRC_LEN = 103;   // [id .. ack]
}  // namespace off

// float32-Spiegel des Bus_Cmd_flat (Reihenfolge == setup_buses.m).
struct GcsCmdFlat {
    float mocap_pos[3];
    float q_ext[4];
    float p_ref[3];
    float v_ref[3];
    float a_ref[3];
    float j_ref[3];
    float s_ref[3];
    float yaw_ref[3];
    uint8_t estop;
    uint8_t ack;
};

// GcsCmdFlat + id -> 106-B-Frame.
inline void build(const GcsCmdFlat& c, uint8_t id, uint8_t buf[SIZE]) {
    buf[off::SYNC] = SYNC0; buf[off::SYNC + 1] = SYNC1;
    buf[off::ID] = id;
    uint8_t* p = buf + off::PAY;
    auto put3 = [&p](const float v[3]) {
        for (int i = 0; i < 3; ++i) { gcs::detail::put_f32(p, v[i]); p += 4; }
    };
    put3(c.mocap_pos);
    for (int i = 0; i < 4; ++i) { gcs::detail::put_f32(p, c.q_ext[i]); p += 4; }
    put3(c.p_ref); put3(c.v_ref); put3(c.a_ref);
    put3(c.j_ref); put3(c.s_ref); put3(c.yaw_ref);
    buf[off::ESTOP] = c.estop;
    buf[off::ACK] = c.ack;
    buf[off::CRC] = gcs::detail::crc8(buf + off::CRC_BEGIN, off::CRC_LEN);
}

// 106-B-Frame -> GcsCmdFlat + id. Prueft Sync + CRC; false bei Fehler.
inline bool parse(const uint8_t buf[SIZE], GcsCmdFlat& c, uint8_t& id) {
    if (buf[off::SYNC] != SYNC0 || buf[off::SYNC + 1] != SYNC1) return false;
    if (gcs::detail::crc8(buf + off::CRC_BEGIN, off::CRC_LEN) != buf[off::CRC]) return false;
    id = buf[off::ID];
    const uint8_t* p = buf + off::PAY;
    auto get3 = [&p](float v[3]) {
        for (int i = 0; i < 3; ++i) { v[i] = gcs::detail::get_f32(p); p += 4; }
    };
    get3(c.mocap_pos);
    for (int i = 0; i < 4; ++i) { c.q_ext[i] = gcs::detail::get_f32(p); p += 4; }
    get3(c.p_ref); get3(c.v_ref); get3(c.a_ref);
    get3(c.j_ref); get3(c.s_ref); get3(c.yaw_ref);
    c.estop = buf[off::ESTOP];
    c.ack = buf[off::ACK];
    return true;
}

}  // namespace gcsf
#endif  // GCS_FRAME_FLAT_HPP
