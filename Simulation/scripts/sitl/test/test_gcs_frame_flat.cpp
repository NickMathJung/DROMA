// test_gcs_frame_flat.cpp — GS-Frame-Cross-Check der FLATNESS-Variante:
// Simulink-Schreiber == Teensy-Leser.
//
// Golden aus dump_gcs_frame_flat_golden.m (MATLAB pack_gcs_frame_flat). Hier:
// gcsf::parse (gcs_frame_flat.hpp) parst die Bytes und muss die float32-
// gerundeten Bus_Cmd_flat-Werte + id exakt rekonstruieren. Zusaetzlich:
// CRC/Sync fangen Korruption, und der USB->OTA-Vollpfad (parse -> pktf::pack ->
// Assembler) liefert dieselben Werte wie ein direkter pktf-Round-Trip.
#include "gcs_frame_flat.hpp"
#include "mcu_flat_packet.hpp"
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
constexpr int in_id = 0, in_moc = 1, in_qe = 4, in_p = 8, in_v = 11, in_a = 14,
              in_j = 17, in_s = 20, in_yaw = 23, in_estop = 26, in_ack = 27,
              frame = 28, NCOL = 134;
}

static void load_frame(const sitl::Row& r, uint8_t buf[gcsf::SIZE]) {
    for (int i = 0; i < gcsf::SIZE; ++i)
        buf[i] = static_cast<uint8_t>(std::lround(r.v[col::frame + i]));
}

// Parsen rekonstruiert Bus_Cmd_flat (float32-exakt) + id.
TEST(GcsFrameFlat, ParseMatchesGolden) {
    auto rows = sitl::read_csv(gpath("gcs_frame_flat_golden.csv"));
    ASSERT_FALSE(rows.empty());
    for (const auto& r : rows) {
        SCOPED_TRACE(r.id);
        ASSERT_EQ(r.v.size(), static_cast<size_t>(col::NCOL));
        uint8_t buf[gcsf::SIZE]; load_frame(r, buf);
        gcsf::GcsCmdFlat c{}; uint8_t id = 0xFF;
        ASSERT_TRUE(gcsf::parse(buf, c, id)) << "parse (sync/crc) fehlgeschlagen";

        EXPECT_EQ(static_cast<long>(std::lround(r.v[col::in_id])), static_cast<long>(id));
        // float32-exakt: MATLAB single(v) == C++ (float)v (gleiche IEEE-Rundung).
        auto chk3 = [&](int base, const float v[3], const char* nm) {
            for (int i = 0; i < 3; ++i)
                EXPECT_EQ(static_cast<float>(r.v[base + i]), v[i]) << nm << i;
        };
        chk3(col::in_moc, c.mocap_pos, "moc");
        for (int i = 0; i < 4; ++i)
            EXPECT_EQ(static_cast<float>(r.v[col::in_qe + i]), c.q_ext[i]) << "qe" << i;
        chk3(col::in_p,   c.p_ref,   "p");
        chk3(col::in_v,   c.v_ref,   "v");
        chk3(col::in_a,   c.a_ref,   "a");
        chk3(col::in_j,   c.j_ref,   "j");
        chk3(col::in_s,   c.s_ref,   "s");
        chk3(col::in_yaw, c.yaw_ref, "yaw");
        EXPECT_EQ(static_cast<long>(std::lround(r.v[col::in_estop])), static_cast<long>(c.estop)) << "estop";
        EXPECT_EQ(r.v[col::in_ack] > 0.5, c.ack != 0) << "ack";
    }
}

// CRC + Sync fangen Korruption (USB-Resync / Bitfehler).
TEST(GcsFrameFlat, RejectsCorruption) {
    auto rows = sitl::read_csv(gpath("gcs_frame_flat_golden.csv"));
    ASSERT_FALSE(rows.empty());
    for (const auto& r : rows) {
        SCOPED_TRACE(r.id);
        uint8_t buf[gcsf::SIZE]; load_frame(r, buf);
        gcsf::GcsCmdFlat c{}; uint8_t id = 0;
        ASSERT_TRUE(gcsf::parse(buf, c, id));                 // Original ok

        uint8_t b1[gcsf::SIZE]; std::memcpy(b1, buf, gcsf::SIZE);
        b1[gcsf::off::PAY + 10] ^= 0x01;                      // 1 Bit im Payload kippen
        EXPECT_FALSE(gcsf::parse(b1, c, id)) << "CRC muesste Payload-Bitfehler fangen";

        uint8_t b2[gcsf::SIZE]; std::memcpy(b2, buf, gcsf::SIZE);
        b2[gcsf::off::SYNC] = 0x00;                           // Sync zerstoeren
        EXPECT_FALSE(gcsf::parse(b2, c, id)) << "Sync-Mismatch muesste ablehnen";

        uint8_t b3[gcsf::SIZE]; std::memcpy(b3, buf, gcsf::SIZE);
        b3[gcsf::off::CRC] ^= 0xFF;                           // CRC-Byte verfaelschen
        EXPECT_FALSE(gcsf::parse(b3, c, id)) << "CRC-Byte-Fehler muesste ablehnen";
    }
}

// Vollpfad wie im Sende-Teensy: USB-Frame -> gcsf::parse -> widen -> pktf::pack
// -> beide Frames durch den Assembler. Muss dasselbe liefern wie ein direkter
// pktf-Round-Trip auf den float32-gerundeten Werten. Faengt Reihenfolge-/
// Feldzuordnungsfehler in gcs_sender_flat.cpp::widen().
TEST(GcsFrameFlat, UsbToOtaFullPath) {
    auto rows = sitl::read_csv(gpath("gcs_frame_flat_golden.csv"));
    ASSERT_FALSE(rows.empty());
    for (const auto& r : rows) {
        SCOPED_TRACE(r.id);
        uint8_t buf[gcsf::SIZE]; load_frame(r, buf);
        gcsf::GcsCmdFlat g{}; uint8_t id = 0;
        ASSERT_TRUE(gcsf::parse(buf, g, id));

        // == widen() aus gcs_sender_flat.cpp (Feldreihenfolge identisch halten!)
        pktf::CmdFlat c{};
        for (int i = 0; i < 3; ++i) c.mocap_pos[i] = g.mocap_pos[i];
        for (int i = 0; i < 4; ++i) c.q_ext[i]     = g.q_ext[i];
        for (int i = 0; i < 3; ++i) c.p_ref[i]     = g.p_ref[i];
        for (int i = 0; i < 3; ++i) c.v_ref[i]     = g.v_ref[i];
        for (int i = 0; i < 3; ++i) c.a_ref[i]     = g.a_ref[i];
        for (int i = 0; i < 3; ++i) c.j_ref[i]     = g.j_ref[i];
        for (int i = 0; i < 3; ++i) c.s_ref[i]     = g.s_ref[i];
        for (int i = 0; i < 3; ++i) c.yaw_ref[i]   = g.yaw_ref[i];
        c.estop = g.estop; c.ack = (g.ack != 0);

        uint8_t bufA[pktf::SIZE], bufB[pktf::SIZE];
        pktf::pack(c, id, /*seq=*/42, bufA, bufB);

        // Empfaengerseite: Assembler muss beim Paar genau ein Kommando freigeben.
        pktf::Assembler asmb;
        EXPECT_FALSE(asmb.feed(bufA));
        ASSERT_TRUE (asmb.feed(bufB));

        // Referenz: direkter Round-Trip ohne Assembler.
        pktf::CmdFlat ref{};
        pktf::unpack_a(bufA, ref);
        pktf::unpack_b(bufB, ref);

        auto same3 = [&](const double a[3], const double b[3], const char* nm) {
            for (int i = 0; i < 3; ++i) EXPECT_EQ(a[i], b[i]) << nm << i;
        };
        same3(ref.mocap_pos, asmb.cmd.mocap_pos, "moc");
        same3(ref.p_ref,     asmb.cmd.p_ref,     "p");
        same3(ref.v_ref,     asmb.cmd.v_ref,     "v");
        same3(ref.a_ref,     asmb.cmd.a_ref,     "a");
        same3(ref.j_ref,     asmb.cmd.j_ref,     "j");
        same3(ref.s_ref,     asmb.cmd.s_ref,     "s");
        same3(ref.yaw_ref,   asmb.cmd.yaw_ref,   "yaw");
        for (int i = 0; i < 4; ++i) EXPECT_EQ(ref.q_ext[i], asmb.cmd.q_ext[i]) << "qe" << i;
        EXPECT_EQ(ref.estop, asmb.cmd.estop);
        EXPECT_EQ(ref.ack,   asmb.cmd.ack);

        // id-Gate: nur die adressierte Drohne nimmt das Paket an.
        EXPECT_TRUE (pktf::id_matches(bufA, id));
        EXPECT_FALSE(pktf::id_matches(bufA, static_cast<uint8_t>(id ^ 0x01)));
    }
}
