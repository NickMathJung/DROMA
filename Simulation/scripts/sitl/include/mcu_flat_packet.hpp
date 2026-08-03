// mcu_flat_packet.hpp — OTA-Codec der FLATNESS-Variante, Multi-Frame.
//
// Serialisiert Bus_Cmd_flat in ZWEI nRF24-Frames (<=32 B) und zurueck.
// Quantisierung bit-identisch zur MATLAB-Kette link_tx_flat/link_rx_flat
// (scripts/flatness/); Cross-Check: test_link_flat_codec.cpp gegen die
// Golden-CSV aus dump_link_flat_codec_golden.m.
//
// Warum zwei Frames: 21x int16 + q_ext(uint32) + Header = ~48 B Nutzlast,
// die nRF24-Payload traegt 32 B. Aufteilung nach Kritikalitaet:
//   Frame A (25 B): mocap_pos, q_ext, p_ref, v_ref   — Zustands-/Fehlerpfad.
//   Frame B (27 B): a_ref, j_ref, s_ref, yaw_ref     — reine Vorsteuerung.
// Verliert die Strecke nur B, degradiert lediglich das Feedforward (ZOH);
// Mocap/estop kommen weiter durch. estop/ack stehen in BEIDEN Frames.
//
// Pairing gegen Tearing: beide Frames tragen dieselbe seq; der Assembler gibt
// ein Kommando erst frei, wenn A und B MIT GLEICHER seq vorliegen (Reihenfolge
// egal). flags-Bit 3 unterscheidet A(0)/B(1).
//
// Festgelegte Entscheidungen (wie mcu_packet.hpp / Kaskade):
//   * little-endian, int16-Quantisierung qi=clamp(round(v/lsb)), lsb=fs/32767,
//     MATLAB round == half-away-from-zero -> std::lround.
//   * smallest-three-Quat mit reserviertem Codewort 0 = "kein Lagebezug".
//   * fs-Werte MUESSEN mit init_link_flat.m uebereinstimmen (dort dokumentiert).
//
// BEIDE Frames sind 27 B gross. Frame A braucht nur 25 und wird auf 27 gepolstert
// (Bytes 25..26 = 0, reserviert): der nRF24 laeuft mit STATISCHER Payload-Groesse
// (RF24::setPayloadSize), Sender und Empfaenger schreiben/lesen also immer
// dieselbe Byte-Zahl. Unterschiedlich grosse Frames wuerden Dynamic Payloads
// erzwingen — mehr Firmware, kein Gewinn.
//
// Byte-Layout Frame A (27 B):            Byte-Layout Frame B (27 B):
//   [0]      id                            [0]      id
//   [1]      flags (bits[1:0]=estop,       [1]      flags (wie A, bit[3]=1)
//            bit[2]=ack, bit[3]=0)         [2]      seq
//   [2]      seq                           [3..8]   a_ref    3x int16 LE
//   [3..8]   mocap_pos 3x int16 LE         [9..14]  j_ref    3x int16 LE
//   [9..12]  q_ext     uint32 LE (sm3)     [15..20] s_ref    3x int16 LE
//   [13..18] p_ref     3x int16 LE         [21..26] yaw_ref  3x int16 LE
//   [19..24] v_ref     3x int16 LE
//   [25..26] reserviert (0)
#ifndef MCU_FLAT_PACKET_HPP
#define MCU_FLAT_PACKET_HPP

#include <cstdint>
#include <cmath>
#include "mcu_packet.hpp"   // pkt::detail — LE-Bytes, quantize, sm3 (eine Quelle)

namespace pktf {

// ---- Paketgeometrie ---------------------------------------------------------
// Eine gemeinsame Groesse fuer beide Frame-Typen (statische nRF-Payload).
constexpr int SIZE   = 27;
constexpr int SIZE_A = SIZE;   // Alias: Frame A nutzt 25 B + 2 B Padding
constexpr int SIZE_B = SIZE;
constexpr int PAD_A  = 25;     // ab hier ist Frame A reserviert/genullt
namespace off {
constexpr int ID = 0, FLAGS = 1, SEQ = 2;             // gemeinsamer Header
constexpr int MOC = 3, QE = 9, P = 13, V = 19;        // Frame A
constexpr int A = 3, J = 9, S = 15, Y = 21;           // Frame B
}  // namespace off
constexpr uint8_t FLAG_FRAME_B = 0x08;                // flags-Bit 3

// ---- int16-Skalen (muessen zu init_link_flat.m passen) ----------------------
constexpr double FS_MOC  = 20.0;    // mocap_pos [m]
constexpr double FS_PREF = 20.0;    // p_ref     [m]
constexpr double FS_VREF = 20.0;    // v_ref     [m/s]
constexpr double FS_AREF = 50.0;    // a_ref     [m/s^2]
constexpr double FS_JREF = 200.0;   // j_ref     [m/s^3]
constexpr double FS_SREF = 2000.0;  // s_ref     [m/s^4]
constexpr double FS_YAW[3] = {4.0, 20.0, 200.0};  // [rad; rad/s; rad/s^2]

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

inline uint8_t make_flags(const CmdFlat& c, bool frame_b) {
    return static_cast<uint8_t>((c.estop & 0x03) | (c.ack ? 0x04 : 0x00) |
                                (frame_b ? FLAG_FRAME_B : 0x00));
}
inline void put_vec3(uint8_t* p, const double v[3], double fs) {
    for (int i = 0; i < 3; ++i) put_i16(p + 2*i, quantize(v[i], fs));
}
inline void get_vec3(const uint8_t* p, double v[3], double fs) {
    for (int i = 0; i < 3; ++i) v[i] = dequantize(get_i16(p + 2*i), fs);
}
}  // namespace detail

// --- API ---------------------------------------------------------------------

// Bus_Cmd_flat + id/seq -> beide OTA-Puffer (gleiche seq = ein Kommandosatz).
inline void pack(const CmdFlat& c, uint8_t id, uint8_t seq,
                 uint8_t bufA[SIZE_A], uint8_t bufB[SIZE_B]) {
    bufA[off::ID] = id;  bufA[off::FLAGS] = detail::make_flags(c, false);  bufA[off::SEQ] = seq;
    detail::put_vec3(bufA + off::MOC, c.mocap_pos, FS_MOC);
    detail::put_u32 (bufA + off::QE,  detail::pack_quat(c.q_ext));
    detail::put_vec3(bufA + off::P,   c.p_ref, FS_PREF);
    detail::put_vec3(bufA + off::V,   c.v_ref, FS_VREF);
    for (int i = PAD_A; i < SIZE; ++i) bufA[i] = 0;   // Padding deterministisch nullen

    bufB[off::ID] = id;  bufB[off::FLAGS] = detail::make_flags(c, true);   bufB[off::SEQ] = seq;
    detail::put_vec3(bufB + off::A, c.a_ref, FS_AREF);
    detail::put_vec3(bufB + off::J, c.j_ref, FS_JREF);
    detail::put_vec3(bufB + off::S, c.s_ref, FS_SREF);
    for (int i = 0; i < 3; ++i)
        detail::put_i16(bufB + off::Y + 2*i, detail::quantize(c.yaw_ref[i], FS_YAW[i]));
}

// Header-Zugriff (beide Frame-Typen identisch).
inline bool    id_matches(const uint8_t* buf, uint8_t own_id) { return buf[off::ID] == own_id; }
inline uint8_t seq_of(const uint8_t* buf)      { return buf[off::SEQ]; }
inline bool    is_frame_b(const uint8_t* buf)  { return (buf[off::FLAGS] & FLAG_FRAME_B) != 0; }

// Frame A -> CmdFlat (fuellt mocap/q_ext/p_ref/v_ref + estop/ack).
inline void unpack_a(const uint8_t bufA[SIZE_A], CmdFlat& c) {
    uint8_t f = bufA[off::FLAGS];
    c.estop = static_cast<uint8_t>(f & 0x03);
    c.ack   = ((f >> 2) & 0x01) != 0;
    detail::get_vec3(bufA + off::MOC, c.mocap_pos, FS_MOC);
    detail::unpack_quat(detail::get_u32(bufA + off::QE), c.q_ext);
    detail::get_vec3(bufA + off::P, c.p_ref, FS_PREF);
    detail::get_vec3(bufA + off::V, c.v_ref, FS_VREF);
}

// Frame B -> CmdFlat (fuellt a/j/s/yaw_ref + estop/ack).
inline void unpack_b(const uint8_t bufB[SIZE_B], CmdFlat& c) {
    uint8_t f = bufB[off::FLAGS];
    c.estop = static_cast<uint8_t>(f & 0x03);
    c.ack   = ((f >> 2) & 0x01) != 0;
    detail::get_vec3(bufB + off::A, c.a_ref, FS_AREF);
    detail::get_vec3(bufB + off::J, c.j_ref, FS_JREF);
    detail::get_vec3(bufB + off::S, c.s_ref, FS_SREF);
    for (int i = 0; i < 3; ++i)
        c.yaw_ref[i] = detail::dequantize(detail::get_i16(bufB + off::Y + 2*i), FS_YAW[i]);
}

// --- Assembler (Firmware-Komfort, drone_hal) ---------------------------------
// Haelt je den letzten A-/B-Frame und gibt ein koherentes CmdFlat frei, sobald
// beide MIT GLEICHER seq da sind (Reihenfolge egal, doppelte Frames harmlos).
// estop/ack werden zusaetzlich aus JEDEM ankommenden Frame sofort uebernommen
// (Safety-Pfad wartet nicht auf das Pairing).
struct Assembler {
    CmdFlat cmd{};            // letztes koherentes Kommando (nach erstem complete)
    uint8_t seq_a = 0, seq_b = 0;
    bool have_a = false, have_b = false;
    uint8_t bufA[SIZE_A] = {0};
    uint8_t bufB[SIZE_B] = {0};

    // Frame einspeisen. true = cmd wurde soeben mit einem koherenten Paar
    // (gleiche seq) aktualisiert.
    bool feed(const uint8_t* buf) {
        uint8_t f = buf[off::FLAGS];
        cmd.estop = static_cast<uint8_t>(f & 0x03);      // Safety sofort
        cmd.ack   = ((f >> 2) & 0x01) != 0;
        if (is_frame_b(buf)) {
            for (int i = 0; i < SIZE_B; ++i) bufB[i] = buf[i];
            seq_b = seq_of(buf); have_b = true;
        } else {
            for (int i = 0; i < SIZE_A; ++i) bufA[i] = buf[i];
            seq_a = seq_of(buf); have_a = true;
        }
        if (have_a && have_b && seq_a == seq_b) {
            unpack_a(bufA, cmd);
            unpack_b(bufB, cmd);
            return true;
        }
        return false;
    }
};

}  // namespace pktf
#endif  // MCU_FLAT_PACKET_HPP
