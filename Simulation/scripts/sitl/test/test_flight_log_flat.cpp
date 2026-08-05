// Gate B fuer das Blackbox-Format (include/flight_log_flat.hpp).
//
// Zwei Dinge koennen hier still danebengehen und waeren im Log erst dann zu
// merken, wenn die Messung schon verloren ist:
//   1) Record-/Header-Groessen: der MATLAB-Leser adressiert mit festen Offsets.
//      Ein eingefuegtes Feld ohne Versionswechsel verschiebt alles.
//   2) Die Ring-Chronologie nach dem Ueberlauf. Vor dem Wrap ist die Reihenfolge
//      trivial, danach muss chrono() ab dem Schreibkopf umlaufen. Genau dieser
//      Fall tritt bei jedem Flug ein, der laenger als 25 s dauert.
#include <cstdint>
#include <vector>
#include <gtest/gtest.h>
#include "flight_log_flat.hpp"

TEST(FlightLogFlat, LayoutIsFrozen) {
    EXPECT_EQ(12u, sizeof(flog::RecFast));
    EXPECT_EQ(76u, sizeof(flog::RecSlow));
    EXPECT_EQ(64u, sizeof(flog::Header));
    // Offsets, auf die der Leser baut
    EXPECT_EQ(0u,  offsetof(flog::RecSlow, tick));
    EXPECT_EQ(4u,  offsetof(flog::RecSlow, mocap_pos));
    EXPECT_EQ(16u, offsetof(flog::RecSlow, p_ref));
    EXPECT_EQ(28u, offsetof(flog::RecSlow, q_ext));
    EXPECT_EQ(44u, offsetof(flog::RecSlow, thr));
    EXPECT_EQ(52u, offsetof(flog::RecSlow, batt_count));
    EXPECT_EQ(54u, offsetof(flog::RecSlow, estop));
    EXPECT_EQ(58u, offsetof(flog::RecSlow, k_hat));
    EXPECT_EQ(60u, offsetof(flog::RecSlow, F));
    EXPECT_EQ(62u, offsetof(flog::RecSlow, aint));
    EXPECT_EQ(68u, offsetof(flog::RecSlow, ufb));
    EXPECT_EQ(74u, offsetof(flog::RecSlow, w_adapt));
}

TEST(FlightLogFlat, ControllerStateScalesCoverTheirRanges) {
    // Die Bereiche stammen aus flatness_ctrl (AINT_MAX 400, UFB_MAX 700) und
    // flatness_khat (k_hat auf [0.5,1.5] geklemmt). Klemmt das Log frueher als der
    // Regler, wuerde genau der interessante Fall — die Saettigung — unsichtbar.
    EXPECT_LT(flog::q15(1.5,    flog::KHAT_SCALE), 32767) << "k_hat-Obergrenze klemmt";
    EXPECT_GT(flog::q15(0.5,    flog::KHAT_SCALE), -32768);
    EXPECT_LT(flog::q15(40.0,   flog::F_SCALE),    32767) << "40 N Schub klemmt";
    EXPECT_LT(flog::q15(400.0,  flog::AINT_SCALE), 32767) << "AINT_MAX klemmt";
    EXPECT_LT(flog::q15(-400.0, flog::AINT_SCALE) * -1, 32769);
    // u_fb_raw wird bewusst UNBEGRENZT geloggt: erst daran sieht man, wie weit
    // ueber UFB_MAX der Regler eigentlich stellen wollte.
    EXPECT_LT(flog::q15(4000.0, flog::UFB_SCALE),  32767) << "5.7x UFB_MAX muss passen";
    // Aufloesung: mindestens 1e-3 der jeweiligen Vollskala
    EXPECT_LT(1.0 / flog::KHAT_SCALE, 1e-3);
    EXPECT_LT(1.0 / flog::F_SCALE,    1e-2);
    EXPECT_LT(1.0 / flog::AINT_SCALE, 0.4);
    EXPECT_LT(1.0 / flog::UFB_SCALE,  0.7);
    // w ist eine Freigabe in [0,1] -- beide Enden muessen exakt darstellbar sein,
    // sonst laesst sich "voll frei" im Log nicht von "fast frei" unterscheiden.
    EXPECT_EQ(0,     flog::q15(0.0, flog::W_SCALE));
    EXPECT_EQ(20000, flog::q15(1.0, flog::W_SCALE));
    EXPECT_LT(1.0 / flog::W_SCALE, 1e-3);
}

TEST(FlightLogFlat, HeaderRoundTrip) {
    flog::Header h;
    flog::fill_header(h, 2, 2, 25000, 2500, 1001, 26000, 123456);
    EXPECT_TRUE(flog::header_valid(h));
    EXPECT_EQ(25000u, h.n_fast);
    EXPECT_EQ(2500u,  h.n_slow);
    EXPECT_EQ(1000u,  h.ts_fast_us);
    EXPECT_EQ(10u,    h.div_slow);
    EXPECT_FLOAT_EQ(flog::GYRO_SCALE, h.gyro_scale);
    flog::Header bad = h; bad.magic[0] = 'X';
    EXPECT_FALSE(flog::header_valid(bad));
    flog::Header bad2 = h; bad2.version = 99;
    EXPECT_FALSE(flog::header_valid(bad2));
}

TEST(FlightLogFlat, FixedPointKeepsSensorResolution) {
    // Ein LSB des Logs darf nicht groeber sein als ein LSB des Sensors, sonst
    // verlieren wir durch die int16-Ablage Aufloesung, die die IMU liefert.
    // Erwartung ist Gleichstand: die Skalen spiegeln die Sensorskalen exakt.
    const double gyro_lsb = (1.0 / 65.5) * 3.14159265358979323846 / 180.0; // rad/s
    const double acc_lsb  = (1.0 / 8192.0) * 9.80665;                      // m/s^2
    EXPECT_LE(1.0 / flog::GYRO_SCALE, gyro_lsb * (1.0 + 1e-6));
    EXPECT_LE(1.0 / flog::ACC_SCALE,  acc_lsb  * (1.0 + 1e-6));
    EXPECT_NEAR(1.0 / flog::GYRO_SCALE, gyro_lsb, 1e-6 * gyro_lsb);
    EXPECT_NEAR(1.0 / flog::ACC_SCALE,  acc_lsb,  1e-6 * acc_lsb);
    // Sattelpunkte: der Messbereich muss ohne Klemmen abgedeckt sein.
    EXPECT_EQ(32767, flog::q15(+100.0, flog::GYRO_SCALE));   // klemmt sauber
    EXPECT_EQ(-32768, flog::q15(-100.0, flog::GYRO_SCALE));
    // Das Log darf nicht FRUEHER klemmen als der Sensor. Am Vollausschlag selbst
    // liegen beide auf derselben Schiene (32767) -- das ist gewollt, nicht ein Fehler.
    const double fsr_gyro = 500.0 * 3.14159265358979323846 / 180.0;        // 8.73 rad/s
    const double fsr_acc  = 4.0 * 9.80665;                                 // 39.2 m/s^2
    EXPECT_LT(flog::q15(0.999 * fsr_gyro, flog::GYRO_SCALE), 32767);
    EXPECT_LT(flog::q15(0.999 * fsr_acc,  flog::ACC_SCALE),  32767);
    EXPECT_LE(flog::q15(fsr_gyro, flog::GYRO_SCALE), 32767);
    EXPECT_LE(flog::q15(fsr_acc,  flog::ACC_SCALE),  32767);
    // Hin und zurueck
    for (double v : {0.0, 0.5, -0.5, 3.25, -8.0}) {
        EXPECT_NEAR(v, flog::unq15(flog::q15(v, flog::GYRO_SCALE), flog::GYRO_SCALE), gyro_lsb);
    }
}

// Hilfsring mit Fast-Records, in denen gyro[0] den Zaehler traegt.
static flog::RecFast mk(int i) { flog::RecFast r{}; r.gyro[0] = (int16_t)i; return r; }

TEST(FlightLogFlat, RingChronologyBeforeWrap) {
    std::vector<flog::RecFast> mem(8);
    flog::Ring<flog::RecFast> r; r.init(mem.data(), 8);
    for (int i = 0; i < 5; ++i) r.push(mk(i));
    ASSERT_EQ(5u, r.n);
    EXPECT_FALSE(r.wrapped());
    for (uint32_t i = 0; i < r.n; ++i) {
        EXPECT_EQ((int16_t)i, r.buf[r.chrono(i)].gyro[0]) << "Position " << i;
        EXPECT_EQ(i, r.seq_of(i));
    }
}

TEST(FlightLogFlat, RingChronologyAfterWrap) {
    std::vector<flog::RecFast> mem(8);
    flog::Ring<flog::RecFast> r; r.init(mem.data(), 8);
    for (int i = 0; i < 21; ++i) r.push(mk(i));   // 2.6 Umlaeufe
    ASSERT_EQ(8u, r.n);
    EXPECT_TRUE(r.wrapped());
    // Die letzten 8: 13..20, chronologisch aufsteigend
    for (uint32_t i = 0; i < r.n; ++i) {
        EXPECT_EQ((int16_t)(13 + i), r.buf[r.chrono(i)].gyro[0]) << "Position " << i;
        EXPECT_EQ(13u + i, r.seq_of(i));
    }
}

TEST(FlightLogFlat, RingExactlyFullDoesNotReorder) {
    // Randfall n == cap ohne echten Ueberlauf: start muss 0 bleiben, sonst
    // rotiert der Dump um einen Record.
    std::vector<flog::RecFast> mem(8);
    flog::Ring<flog::RecFast> r; r.init(mem.data(), 8);
    for (int i = 0; i < 8; ++i) r.push(mk(i));
    ASSERT_EQ(8u, r.n);
    EXPECT_FALSE(r.wrapped()) << "genau voll ist noch kein Ueberlauf";
    for (uint32_t i = 0; i < r.n; ++i)
        EXPECT_EQ((int16_t)i, r.buf[r.chrono(i)].gyro[0]) << "Position " << i;
}

TEST(FlightLogFlat, ThrottleQuantisationCoversFullRange) {
    // Die Firmware legt throttle als 0.01-%-Festkomma ab (0..10000).
    auto pack = [](double th_pct) -> uint16_t {
        double t = th_pct * 100.0;
        return (uint16_t)(t < 0 ? 0 : (t > 10000 ? 10000 : t + 0.5));
    };
    EXPECT_EQ(0u,     pack(0.0));
    EXPECT_EQ(4900u,  pack(49.0));
    EXPECT_EQ(10000u, pack(100.0));
    EXPECT_EQ(10000u, pack(123.0));   // klemmt
    EXPECT_EQ(0u,     pack(-5.0));
}
