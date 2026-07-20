// test_link_codec.cpp — Codec-Cross-Check: MATLAB link_tx/link_rx == C++ pkt.
//
// Golden aus dump_link_codec_golden.m (data/link_codec_golden.csv). Pro Zeile
// ein Bus_Cmd durch die MATLAB-Kette; hier durch pkt::pack/pkt::unpack.
//
//   L1 (Wire):   int16[7], uint32[3] (sm3), flags  bit-exakt gegen pkt::pack.
//   L2 (decode): F/Om/tau bit-exakt, Quats tol 1e-12 gegen pkt::unpack.
//   + id/seq: reiner C++-Round-Trip (die MATLAB-Kette traegt sie nicht).
//
// Schliesst "Sim == HW" fuer den OTA-Codec formal.
#include "mcu_packet.hpp"
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

// --- Spaltenindizes in Row.v (0-basiert, == Header von dump_link_codec_golden.m)
namespace col {
constexpr int in_F = 0, in_qd = 1, in_qr = 5, in_Om = 9, in_tr = 12, in_qe = 15,
              in_estop = 19, in_ack = 20;
constexpr int tx_i16 = 21, tx_q = 28, tx_flags = 31;
constexpr int rx_F = 33, rx_qd = 34, rx_qr = 38, rx_Om = 42, rx_tr = 45,
              rx_qe = 48, rx_estop = 52, rx_ack = 53;
constexpr int NCOL = 54;
}  // namespace col

static pkt::Cmd cmd_from_row(const sitl::Row& r) {
    pkt::Cmd c{};
    c.F_des = r.v[col::in_F];
    for (int i = 0; i < 4; ++i) c.q_des[i] = r.v[col::in_qd + i];
    for (int i = 0; i < 4; ++i) c.q_ref[i] = r.v[col::in_qr + i];
    for (int i = 0; i < 3; ++i) c.Omega_ref[i] = r.v[col::in_Om + i];
    for (int i = 0; i < 3; ++i) c.tau_ref[i] = r.v[col::in_tr + i];
    for (int i = 0; i < 4; ++i) c.q_ext[i] = r.v[col::in_qe + i];
    c.estop = static_cast<uint8_t>(std::lround(r.v[col::in_estop]));
    c.ack = r.v[col::in_ack] > 0.5;
    return c;
}

// L1: gepackte Wire-Werte bit-identisch zu MATLAB link_tx.
TEST(LinkCodec, WireBitExact) {
    auto rows = sitl::read_csv(gpath("link_codec_golden.csv"));
    ASSERT_FALSE(rows.empty());
    for (const auto& r : rows) {
        SCOPED_TRACE(r.id);
        ASSERT_EQ(r.v.size(), static_cast<size_t>(col::NCOL));
        pkt::Cmd c = cmd_from_row(r);
        uint8_t buf[pkt::SIZE];
        pkt::pack(c, /*id=*/0xA5, /*seq=*/0x00, buf);

        // int16-Teil: tx_i16 = [F | Om1 Om2 Om3 | tau1 tau2 tau3].
        int16_t wire_i16[7];
        wire_i16[0] = pkt::detail::get_i16(buf + pkt::off::F);
        for (int i = 0; i < 3; ++i) wire_i16[1 + i] = pkt::detail::get_i16(buf + pkt::off::OM + 2 * i);
        for (int i = 0; i < 3; ++i) wire_i16[4 + i] = pkt::detail::get_i16(buf + pkt::off::TAU + 2 * i);
        for (int k = 0; k < 7; ++k)
            EXPECT_EQ(static_cast<long>(std::llround(r.v[col::tx_i16 + k])),
                      static_cast<long>(wire_i16[k])) << "  i16[" << k << "]";

        // sm3-Quats: tx_q = [q_des q_ref q_ext].
        uint32_t wire_q[3] = {pkt::detail::get_u32(buf + pkt::off::QD),
                              pkt::detail::get_u32(buf + pkt::off::QR),
                              pkt::detail::get_u32(buf + pkt::off::QE)};
        for (int k = 0; k < 3; ++k)
            EXPECT_EQ(static_cast<uint32_t>(r.v[col::tx_q + k]), wire_q[k]) << "  q[" << k << "]";

        // flags: bits[1:0]=estop, bit[2]=ack.
        uint8_t f = buf[pkt::off::FLAGS];
        EXPECT_EQ(static_cast<long>(std::llround(r.v[col::tx_flags + 0])), static_cast<long>(f & 0x03)) << " estop";
        EXPECT_EQ(r.v[col::tx_flags + 1] > 0.5, ((f >> 2) & 0x01) != 0) << " ack";
    }
}

// L2: entpacktes Bus_Cmd == MATLAB link_rx (Skalare exakt, Quats tol).
TEST(LinkCodec, DecodeMatchesRx) {
    auto rows = sitl::read_csv(gpath("link_codec_golden.csv"));
    ASSERT_FALSE(rows.empty());
    double worst_q = 0.0; std::string worst_id;
    for (const auto& r : rows) {
        SCOPED_TRACE(r.id);
        pkt::Cmd c = cmd_from_row(r);
        uint8_t buf[pkt::SIZE];
        pkt::pack(c, 0xA5, 0x00, buf);
        pkt::Cmd d{}; uint8_t id = 0, seq = 0;
        pkt::unpack(buf, d, id, seq);

        // F/Om/tau: identische double-Ops (p .* fs/qmax) -> bit-exakt.
        EXPECT_EQ(r.v[col::rx_F], d.F_des) << " F_des";
        for (int i = 0; i < 3; ++i) EXPECT_EQ(r.v[col::rx_Om + i], d.Omega_ref[i]) << " Om" << i;
        for (int i = 0; i < 3; ++i) EXPECT_EQ(r.v[col::rx_tr + i], d.tau_ref[i]) << " tau" << i;

        // Quats: sm3-Decode nutzt sqrt (libm) -> tol.
        auto chk = [&](int base, const double q[4], const char* nm) {
            for (int i = 0; i < 4; ++i) {
                double diff = std::fabs(r.v[base + i] - q[i]);
                if (diff > worst_q) { worst_q = diff; worst_id = r.id + std::string("/") + nm; }
                EXPECT_LE(diff, kQuatTol) << nm << "[" << i << "]";
            }
        };
        chk(col::rx_qd, d.q_des, "q_des");
        chk(col::rx_qr, d.q_ref, "q_ref");
        chk(col::rx_qe, d.q_ext, "q_ext");

        EXPECT_EQ(static_cast<long>(std::llround(r.v[col::rx_estop])), static_cast<long>(d.estop)) << " estop";
        EXPECT_EQ(r.v[col::rx_ack] > 0.5, d.ack) << " ack";
    }
    RecordProperty("worst_quat_abs_diff", std::to_string(worst_q));
    if (!worst_id.empty())
        std::fprintf(stderr, "[ INFO     ] groesste Quat-Abweichung %.3e bei %s\n", worst_q, worst_id.c_str());
}

// id/seq: reiner C++-Round-Trip (nicht Teil der MATLAB-Kette).
TEST(LinkCodec, HeaderRoundTrip) {
    auto rows = sitl::read_csv(gpath("link_codec_golden.csv"));
    ASSERT_FALSE(rows.empty());
    int n = 0;
    for (const auto& r : rows) {
        pkt::Cmd c = cmd_from_row(r);
        uint8_t id = static_cast<uint8_t>(0x10 + (n % 3));   // 3 Drohnen-IDs
        uint8_t seq = static_cast<uint8_t>(n & 0xFF);
        uint8_t buf[pkt::SIZE];
        pkt::pack(c, id, seq, buf);
        pkt::Cmd d{}; uint8_t id2 = 0, seq2 = 0;
        pkt::unpack(buf, d, id2, seq2);
        EXPECT_EQ(id, id2);
        EXPECT_EQ(seq, seq2);
        ++n;
    }
}
