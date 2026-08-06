// test_link_flat_codec.cpp — Codec-Cross-Check der FLATNESS-Variante:
// MATLAB link_tx_flat/link_rx_flat == C++ pktf (2-Frame-OTA).
//
// Golden aus dump_link_flat_codec_golden.m (data/link_flat_codec_golden.csv).
// Pro Zeile ein Bus_Cmd_flat durch die MATLAB-Kette; hier durch
// pktf::pack + unpack_a/unpack_b.
//
//   L1 (Wire):   int16[21], uint32 (sm3 q_ext), flags bit-exakt gegen pktf::pack.
//   L2 (decode): Vektoren bit-exakt, q_ext tol 1e-12.
//   + Frame-Geometrie (<=32 B), Assembler-Pairing, id/seq-Round-Trip.
//
// Schliesst "Sim == HW" fuer den Flatness-OTA-Codec formal.
#include "mcu_flat_packet.hpp"
#include "csv.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <string>

#ifndef GOLDEN_DIR
#define GOLDEN_DIR "."
#endif
static std::string gpath(const char* f) { return std::string(GOLDEN_DIR) + "/" + f; }
static constexpr double kQuatTol = 1e-12;

// --- Spaltenindizes (0-basiert, == Header von dump_link_flat_codec_golden.m) --
namespace col {
constexpr int in_moc = 0, in_qe = 3, in_p = 7, in_v = 10, in_a = 13, in_j = 16,
              in_s = 19, in_yaw = 22, in_estop = 25, in_ack = 26;
constexpr int tx_i16 = 27, tx_q = 48, tx_flags = 49;
constexpr int rx_moc = 51, rx_qe = 54, rx_p = 58, rx_v = 61, rx_a = 64,
              rx_j = 67, rx_s = 70, rx_yaw = 73, rx_estop = 76, rx_ack = 77;
constexpr int NCOL = 78;
}  // namespace col

static pktf::CmdFlat cmd_from_row(const sitl::Row& r) {
    pktf::CmdFlat c{};
    for (int i = 0; i < 3; ++i) c.mocap_pos[i] = r.v[col::in_moc + i];
    for (int i = 0; i < 4; ++i) c.q_ext[i]     = r.v[col::in_qe  + i];
    for (int i = 0; i < 3; ++i) c.p_ref[i]     = r.v[col::in_p   + i];
    for (int i = 0; i < 3; ++i) c.v_ref[i]     = r.v[col::in_v   + i];
    for (int i = 0; i < 3; ++i) c.a_ref[i]     = r.v[col::in_a   + i];
    for (int i = 0; i < 3; ++i) c.j_ref[i]     = r.v[col::in_j   + i];
    for (int i = 0; i < 3; ++i) c.s_ref[i]     = r.v[col::in_s   + i];
    for (int i = 0; i < 3; ++i) c.yaw_ref[i]   = r.v[col::in_yaw + i];
    c.estop = static_cast<uint8_t>(std::lround(r.v[col::in_estop]));
    c.ack   = r.v[col::in_ack] > 0.5;
    return c;
}

// Die 21 Wire-int16 in MATLAB-Reihenfolge aus beiden Frames einsammeln:
// [mocap(3) | p(3) | v(3) | a(3) | j(3) | s(3) | yaw(3)]
static void wire_i16_from_frames(const uint8_t* bufA, const uint8_t* bufB, int16_t out[21]) {
    using namespace pktf;
    for (int i = 0; i < 3; ++i) out[0  + i] = detail::get_i16(bufA + off::MOC + 2*i);
    for (int i = 0; i < 3; ++i) out[3  + i] = detail::get_i16(bufA + off::P   + 2*i);
    for (int i = 0; i < 3; ++i) out[6  + i] = detail::get_i16(bufA + off::V   + 2*i);
    for (int i = 0; i < 3; ++i) out[9  + i] = detail::get_i16(bufB + off::A   + 2*i);
    for (int i = 0; i < 3; ++i) out[12 + i] = detail::get_i16(bufB + off::J   + 2*i);
    for (int i = 0; i < 3; ++i) out[15 + i] = detail::get_i16(bufB + off::S   + 2*i);
    for (int i = 0; i < 3; ++i) out[18 + i] = detail::get_i16(bufB + off::Y   + 2*i);
}

// Beide Frames muessen in die nRF24-Payload passen (32 B).
TEST(LinkFlatCodec, FramesFitNrfPayload) {
    EXPECT_LE(pktf::SIZE_A, 32) << "Frame A passt nicht in die nRF24-Payload";
    EXPECT_LE(pktf::SIZE_B, 32) << "Frame B passt nicht in die nRF24-Payload";
}

// L1: gepackte Wire-Werte bit-identisch zu MATLAB link_tx_flat.
TEST(LinkFlatCodec, WireBitExact) {
    auto rows = sitl::read_csv(gpath("link_flat_codec_golden.csv"));
    ASSERT_FALSE(rows.empty());
    for (const auto& r : rows) {
        SCOPED_TRACE(r.id);
        ASSERT_EQ(r.v.size(), static_cast<size_t>(col::NCOL));
        pktf::CmdFlat c = cmd_from_row(r);
        uint8_t bufA[pktf::SIZE_A], bufB[pktf::SIZE_B];
        pktf::pack(c, /*id=*/0xA5, /*seq=*/0x00, bufA, bufB);

        int16_t wire[21];
        wire_i16_from_frames(bufA, bufB, wire);
        for (int k = 0; k < 21; ++k)
            EXPECT_EQ(static_cast<long>(std::llround(r.v[col::tx_i16 + k])),
                      static_cast<long>(wire[k])) << "  i16[" << k << "]";

        // sm3-Quat q_ext (Frame A).
        EXPECT_EQ(static_cast<uint32_t>(r.v[col::tx_q]),
                  pktf::detail::get_u32(bufA + pktf::off::QE)) << " q_ext-Code";

        // flags: bits[1:0]=estop, bit[2]=ack — in BEIDEN Frames identisch;
        // bit[3] unterscheidet A/B.
        for (const uint8_t* b : {static_cast<const uint8_t*>(bufA), static_cast<const uint8_t*>(bufB)}) {
            uint8_t f = b[pktf::off::FLAGS];
            EXPECT_EQ(static_cast<long>(std::llround(r.v[col::tx_flags + 0])),
                      static_cast<long>(f & 0x03)) << " estop";
            EXPECT_EQ(r.v[col::tx_flags + 1] > 0.5, ((f >> 2) & 0x01) != 0) << " ack";
        }
        EXPECT_FALSE(pktf::is_frame_b(bufA)) << " Frame A darf Bit 3 nicht setzen";
        EXPECT_TRUE (pktf::is_frame_b(bufB)) << " Frame B muss Bit 3 setzen";
    }
}

// L2: entpacktes Bus_Cmd_flat == MATLAB link_rx_flat (Vektoren exakt, Quat tol).
TEST(LinkFlatCodec, DecodeMatchesRx) {
    auto rows = sitl::read_csv(gpath("link_flat_codec_golden.csv"));
    ASSERT_FALSE(rows.empty());
    double worst_q = 0.0; std::string worst_id;
    for (const auto& r : rows) {
        SCOPED_TRACE(r.id);
        pktf::CmdFlat c = cmd_from_row(r);
        uint8_t bufA[pktf::SIZE_A], bufB[pktf::SIZE_B];
        pktf::pack(c, 0xA5, 0x00, bufA, bufB);
        pktf::CmdFlat d{};
        pktf::unpack_a(bufA, d);
        pktf::unpack_b(bufB, d);

        // Vektoren: identische double-Ops (p .* fs/qmax) -> bit-exakt.
        auto chk3 = [&](int base, const double v[3], const char* nm) {
            for (int i = 0; i < 3; ++i) EXPECT_EQ(r.v[base + i], v[i]) << nm << "[" << i << "]";
        };
        chk3(col::rx_moc, d.mocap_pos, "mocap_pos");
        chk3(col::rx_p,   d.p_ref,     "p_ref");
        chk3(col::rx_v,   d.v_ref,     "v_ref");
        chk3(col::rx_a,   d.a_ref,     "a_ref");
        chk3(col::rx_j,   d.j_ref,     "j_ref");
        chk3(col::rx_s,   d.s_ref,     "s_ref");
        chk3(col::rx_yaw, d.yaw_ref,   "yaw_ref");

        // q_ext: sm3-Decode nutzt sqrt (libm) -> tol.
        for (int i = 0; i < 4; ++i) {
            double diff = std::fabs(r.v[col::rx_qe + i] - d.q_ext[i]);
            if (diff > worst_q) { worst_q = diff; worst_id = r.id; }
            EXPECT_LE(diff, kQuatTol) << "q_ext[" << i << "]";
        }

        EXPECT_EQ(static_cast<long>(std::llround(r.v[col::rx_estop])),
                  static_cast<long>(d.estop)) << " estop";
        EXPECT_EQ(r.v[col::rx_ack] > 0.5, d.ack) << " ack";
    }
    RecordProperty("worst_quat_abs_diff", std::to_string(worst_q));
    if (!worst_id.empty())
        std::fprintf(stderr, "[ INFO     ] groesste Quat-Abweichung %.3e bei %s\n", worst_q, worst_id.c_str());
}

// id/seq: Round-Trip ueber beide Frames (nicht Teil der MATLAB-Kette).
TEST(LinkFlatCodec, HeaderRoundTrip) {
    auto rows = sitl::read_csv(gpath("link_flat_codec_golden.csv"));
    ASSERT_FALSE(rows.empty());
    int n = 0;
    for (const auto& r : rows) {
        pktf::CmdFlat c = cmd_from_row(r);
        uint8_t id  = static_cast<uint8_t>(0x10 + (n % 3));
        uint8_t seq = static_cast<uint8_t>(n & 0xFF);
        uint8_t bufA[pktf::SIZE_A], bufB[pktf::SIZE_B];
        pktf::pack(c, id, seq, bufA, bufB);
        EXPECT_TRUE(pktf::id_matches(bufA, id));
        EXPECT_TRUE(pktf::id_matches(bufB, id));
        EXPECT_FALSE(pktf::id_matches(bufA, static_cast<uint8_t>(id + 1)));
        EXPECT_EQ(seq, pktf::seq_of(bufA));
        EXPECT_EQ(seq, pktf::seq_of(bufB));
        ++n;
    }
}

// Assembler: gibt erst bei A+B MIT GLEICHER seq frei (Tearing-Schutz),
// Reihenfolge egal; estop/ack kommen sofort aus jedem Frame durch.
TEST(LinkFlatCodec, AssemblerPairsBySeq) {
    pktf::CmdFlat c1{}; c1.q_ext[0] = 1.0;
    c1.mocap_pos[2] = 1.0; c1.p_ref[2] = 1.0; c1.a_ref[0] = 2.0; c1.estop = 0;
    pktf::CmdFlat c2 = c1; c2.mocap_pos[2] = 2.0; c2.a_ref[0] = 5.0;

    uint8_t a1[pktf::SIZE_A], b1[pktf::SIZE_B], a2[pktf::SIZE_A], b2[pktf::SIZE_B];
    pktf::pack(c1, 0x01, /*seq=*/7, a1, b1);
    pktf::pack(c2, 0x01, /*seq=*/8, a2, b2);

    // Toleranz > lsb (mocap 20/32767, a_ref 50/32767), aber weit unter dem
    // Abstand der Testwerte: hier zaehlt WELCHER Satz ankommt, nicht die
    // Quantisierungsgenauigkeit (die deckt DecodeMatchesRx bit-exakt ab).
    constexpr double kQ = 5e-3;
    pktf::Assembler asmb;
    EXPECT_FALSE(asmb.feed(a1)) << "einzelnes A darf nicht freigeben";
    EXPECT_TRUE (asmb.feed(b1)) << "A+B mit gleicher seq muss freigeben";
    EXPECT_NEAR(1.0, asmb.cmd.mocap_pos[2], kQ);
    EXPECT_NEAR(2.0, asmb.cmd.a_ref[0], kQ);

    // Neues A (seq 8) gegen altes B (seq 7): darf NICHT freigeben (Tearing).
    EXPECT_FALSE(asmb.feed(a2)) << "seq-Mismatch darf nicht freigeben";
    EXPECT_NEAR(2.0, asmb.cmd.a_ref[0], kQ) << "cmd muss beim letzten koherenten Satz bleiben";
    EXPECT_TRUE(asmb.feed(b2));
    EXPECT_NEAR(2.0, asmb.cmd.mocap_pos[2], kQ);
    EXPECT_NEAR(5.0, asmb.cmd.a_ref[0], kQ);

    // Umgekehrte Reihenfolge (B zuerst) funktioniert genauso.
    pktf::Assembler asmb2;
    EXPECT_FALSE(asmb2.feed(b1));
    EXPECT_TRUE (asmb2.feed(a1));

    // Safety-Pfad: estop kommt aus JEDEM Frame sofort, ohne Pairing.
    pktf::CmdFlat ck = c1; ck.estop = 2; ck.ack = true;
    uint8_t ak[pktf::SIZE_A], bk[pktf::SIZE_B];
    pktf::pack(ck, 0x01, /*seq=*/9, ak, bk);
    pktf::Assembler asmb3;
    asmb3.feed(ak);
    EXPECT_EQ(2, asmb3.cmd.estop) << "estop muss ohne Pairing durchkommen";
    EXPECT_TRUE(asmb3.cmd.ack);
}
