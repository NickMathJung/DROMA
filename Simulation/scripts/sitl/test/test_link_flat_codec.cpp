// test_link_flat_codec.cpp — Codec-Cross-Check der FLATNESS-Variante:
// MATLAB link_tx_flat/link_rx_flat == C++ pktf (ein OTA-Frame je Drohne).
//
// Golden aus dump_link_flat_codec_golden.m (data/link_flat_codec_golden.csv).
// Pro Zeile ein Bus_Cmd_flat durch die MATLAB-Kette; hier durch
// pktf::pack + pktf::unpack.
//
//   L1 (Wire):   int16[13], uint32 (sm3 q_ext), flags bit-exakt gegen pktf::pack.
//   L2 (decode): Vektoren bit-exakt, q_ext tol 1e-12, j/s/Gier-Ableitungen = 0.
//   + Frame-Geometrie (<=32 B), id/seq-Round-Trip.
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
constexpr int tx_i16 = 27, tx_q = 40, tx_flags = 41;
constexpr int rx_moc = 43, rx_qe = 46, rx_p = 50, rx_v = 53, rx_a = 56,
              rx_j = 59, rx_s = 62, rx_yaw = 65, rx_estop = 68, rx_ack = 69;
constexpr int NCOL = 70;
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

// Die 13 Wire-int16 in MATLAB-Reihenfolge: [mocap(3) | p(3) | v(3) | a(3) | yaw(1)]
static void wire_i16_from_frame(const uint8_t* buf, int16_t out[13]) {
    using namespace pktf;
    for (int i = 0; i < 3; ++i) out[0 + i] = detail::get_i16(buf + off::MOC + 2*i);
    for (int i = 0; i < 3; ++i) out[3 + i] = detail::get_i16(buf + off::P   + 2*i);
    for (int i = 0; i < 3; ++i) out[6 + i] = detail::get_i16(buf + off::V   + 2*i);
    for (int i = 0; i < 3; ++i) out[9 + i] = detail::get_i16(buf + off::A   + 2*i);
    out[12] = detail::get_i16(buf + off::Y);
}

// Der Frame muss in die nRF24-Payload passen (32 B).
TEST(LinkFlatCodec, FrameFitsNrfPayload) {
    EXPECT_LE(pktf::SIZE, 32) << "Frame passt nicht in die nRF24-Payload";
    EXPECT_EQ(pktf::off::Y + 2, pktf::SIZE) << "Layout fuellt den Frame nicht exakt";
}

// L1: gepackte Wire-Werte bit-identisch zu MATLAB link_tx_flat.
TEST(LinkFlatCodec, WireBitExact) {
    auto rows = sitl::read_csv(gpath("link_flat_codec_golden.csv"));
    ASSERT_FALSE(rows.empty());
    for (const auto& r : rows) {
        SCOPED_TRACE(r.id);
        ASSERT_EQ(r.v.size(), static_cast<size_t>(col::NCOL));
        pktf::CmdFlat c = cmd_from_row(r);
        uint8_t buf[pktf::SIZE];
        pktf::pack(c, /*id=*/0x05, /*seq=*/0x00, buf);

        int16_t wire[13];
        wire_i16_from_frame(buf, wire);
        for (int k = 0; k < 13; ++k)
            EXPECT_EQ(static_cast<long>(std::llround(r.v[col::tx_i16 + k])),
                      static_cast<long>(wire[k])) << "  i16[" << k << "]";

        EXPECT_EQ(static_cast<uint32_t>(r.v[col::tx_q]),
                  pktf::detail::get_u32(buf + pktf::off::QE)) << " q_ext-Code";

        // hdr: bits[5:4]=estop, bit[6]=ack, bit[7]=0.
        uint8_t h = buf[pktf::off::HDR];
        EXPECT_EQ(static_cast<long>(std::llround(r.v[col::tx_flags + 0])),
                  static_cast<long>((h >> 4) & 0x03)) << " estop";
        EXPECT_EQ(r.v[col::tx_flags + 1] > 0.5, ((h >> 6) & 0x01) != 0) << " ack";
        EXPECT_EQ(0, h & 0x80) << " reserviertes Bit 7";
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
        uint8_t buf[pktf::SIZE];
        pktf::pack(c, 0x05, 0x00, buf);
        pktf::CmdFlat d{};
        for (int i = 0; i < 3; ++i) { d.j_ref[i] = 99.0; d.s_ref[i] = 99.0; d.yaw_ref[i] = 99.0; }
        pktf::unpack(buf, d);

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
        for (int i = 0; i < 3; ++i) {
            EXPECT_EQ(0.0, d.j_ref[i]) << "j_ref wird nicht uebertragen";
            EXPECT_EQ(0.0, d.s_ref[i]) << "s_ref wird nicht uebertragen";
        }
        EXPECT_EQ(0.0, d.yaw_ref[1]);
        EXPECT_EQ(0.0, d.yaw_ref[2]);

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

// id/seq: Round-Trip (nicht Teil der MATLAB-Kette); id und Flags teilen sich hdr.
TEST(LinkFlatCodec, HeaderRoundTrip) {
    auto rows = sitl::read_csv(gpath("link_flat_codec_golden.csv"));
    ASSERT_FALSE(rows.empty());
    int n = 0;
    for (const auto& r : rows) {
        SCOPED_TRACE(r.id);
        pktf::CmdFlat c = cmd_from_row(r);
        uint8_t id  = static_cast<uint8_t>(n % 16);
        uint8_t seq = static_cast<uint8_t>(n & 0xFF);
        uint8_t buf[pktf::SIZE];
        pktf::pack(c, id, seq, buf);
        EXPECT_EQ(id, pktf::id_of(buf)) << "estop/ack duerfen die id nicht verfaelschen";
        EXPECT_TRUE(pktf::id_matches(buf, id));
        EXPECT_FALSE(pktf::id_matches(buf, static_cast<uint8_t>((id + 1) & 0x0F)));
        EXPECT_EQ(seq, pktf::seq_of(buf));
        ++n;
    }
}
