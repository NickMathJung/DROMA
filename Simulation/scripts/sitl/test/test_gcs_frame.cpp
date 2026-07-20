// test_gcs_frame.cpp — GS-Frame-Cross-Check: Simulink-Schreiber == Teensy-Leser.
//
// Golden aus dump_gcs_frame_golden.m (MATLAB pack_gcs_frame). Hier: gcs::parse
// (gcs_frame.hpp) parst die Bytes und muss die float32-gerundeten Bus_Cmd-Werte
// + id exakt rekonstruieren. Zusaetzlich: CRC/Sync fangen Korruption.
#include "gcs_frame.hpp"
#include "csv.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#ifndef GOLDEN_DIR
#define GOLDEN_DIR "."
#endif
static std::string gpath(const char* f) { return std::string(GOLDEN_DIR) + "/" + f; }

namespace col {
constexpr int in_id = 0, in_F = 1, in_qd = 2, in_qr = 6, in_Om = 10, in_tr = 13,
              in_qe = 16, in_estop = 20, in_ack = 21, frame = 22, NCOL = 104;
}

static void load_frame(const sitl::Row& r, uint8_t buf[gcs::SIZE]) {
    for (int i = 0; i < gcs::SIZE; ++i)
        buf[i] = static_cast<uint8_t>(std::lround(r.v[col::frame + i]));
}

// Parsen rekonstruiert Bus_Cmd (float32-exakt) + id.
TEST(GcsFrame, ParseMatchesGolden) {
    auto rows = sitl::read_csv(gpath("gcs_frame_golden.csv"));
    ASSERT_FALSE(rows.empty());
    for (const auto& r : rows) {
        SCOPED_TRACE(r.id);
        ASSERT_EQ(r.v.size(), static_cast<size_t>(col::NCOL));
        uint8_t buf[gcs::SIZE]; load_frame(r, buf);
        gcs::GcsCmd c{}; uint8_t id = 0xFF;
        ASSERT_TRUE(gcs::parse(buf, c, id)) << "parse (sync/crc) fehlgeschlagen";

        EXPECT_EQ(static_cast<long>(std::lround(r.v[col::in_id])), static_cast<long>(id));
        // float32-exakt: MATLAB single(v) == C++ (float)v (gleiche IEEE-Rundung).
        EXPECT_EQ(static_cast<float>(r.v[col::in_F]), c.F_des) << "F_des";
        for (int i = 0; i < 4; ++i) EXPECT_EQ(static_cast<float>(r.v[col::in_qd + i]), c.q_des[i]) << "qd" << i;
        for (int i = 0; i < 4; ++i) EXPECT_EQ(static_cast<float>(r.v[col::in_qr + i]), c.q_ref[i]) << "qr" << i;
        for (int i = 0; i < 3; ++i) EXPECT_EQ(static_cast<float>(r.v[col::in_Om + i]), c.Omega_ref[i]) << "Om" << i;
        for (int i = 0; i < 3; ++i) EXPECT_EQ(static_cast<float>(r.v[col::in_tr + i]), c.tau_ref[i]) << "tr" << i;
        for (int i = 0; i < 4; ++i) EXPECT_EQ(static_cast<float>(r.v[col::in_qe + i]), c.q_ext[i]) << "qe" << i;
        EXPECT_EQ(static_cast<long>(std::lround(r.v[col::in_estop])), static_cast<long>(c.estop)) << "estop";
        EXPECT_EQ(r.v[col::in_ack] > 0.5, c.ack != 0) << "ack";
    }
}

// CRC + Sync fangen Korruption (USB-Resync / Bitfehler).
TEST(GcsFrame, RejectsCorruption) {
    auto rows = sitl::read_csv(gpath("gcs_frame_golden.csv"));
    ASSERT_FALSE(rows.empty());
    for (const auto& r : rows) {
        SCOPED_TRACE(r.id);
        uint8_t buf[gcs::SIZE]; load_frame(r, buf);
        gcs::GcsCmd c{}; uint8_t id = 0;
        ASSERT_TRUE(gcs::parse(buf, c, id));                 // Original ok

        uint8_t b1[gcs::SIZE]; std::memcpy(b1, buf, gcs::SIZE);
        b1[gcs::off::PAY + 10] ^= 0x01;                      // 1 Bit im Payload kippen
        EXPECT_FALSE(gcs::parse(b1, c, id)) << "CRC muesste Payload-Bitfehler fangen";

        uint8_t b2[gcs::SIZE]; std::memcpy(b2, buf, gcs::SIZE);
        b2[gcs::off::SYNC] = 0x00;                           // Sync zerstoeren
        EXPECT_FALSE(gcs::parse(b2, c, id)) << "Sync-Mismatch muesste ablehnen";

        uint8_t b3[gcs::SIZE]; std::memcpy(b3, buf, gcs::SIZE);
        b3[gcs::off::CRC] ^= 0xFF;                           // CRC-Byte verfaelschen
        EXPECT_FALSE(gcs::parse(b3, c, id)) << "CRC-Byte-Fehler muesste ablehnen";
    }
}
