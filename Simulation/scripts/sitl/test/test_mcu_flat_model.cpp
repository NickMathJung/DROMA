// test_mcu_flat_model.cpp — Modell-SITL-Golden fuer die FLATNESS-MCU: der
// generierte MCU_FLAT-Code (mcu_flat.cpp) wird tickweise gegen die in Simulink
// aufgezeichneten I/O (golden_mcu_flat_io.csv, via log_mcu_flat_golden.m)
// gedifft. Pendant zu test_mcu_model.cpp (Kaskade); Safety-Integrationstests
// (estop/overspeed/button/tilt/batterie) sind auf die Bus_Cmd_flat-ExtU
// uebertragen: Hover heisst hier mocap_pos == p_ref (der Flachheitsregler
// erzeugt den Schub selbst; zeta1 startet bei g -> throttle > 0 ohne Kill).
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include "mcu_flat_io.hpp"
#include "throttle_poly.hpp"   // P_THROTTLE (aus params.m via run_mcu_recert.m)

#ifndef GOLDEN_MCU_FLAT_CSV
#define GOLDEN_MCU_FLAT_CSV "data/golden_mcu_flat_io.csv"
#endif
#ifndef GOLDEN_TOL
#define GOLDEN_TOL 1e-9
#endif

using namespace sitl_flat;

static double throttle_eq(double omega) {
    double y = mcuref::P_THROTTLE[0];
    for (int k = 1; k < mcuref::P_THROTTLE_N; ++k) y = y * omega + mcuref::P_THROTTLE[k];
    return y;
}

static const NamedCsv* golden() {
    static bool tried = false; static NamedCsv g; static bool ok = false;
    if (!tried) { tried = true;
        try { g = read_named_csv(GOLDEN_MCU_FLAT_CSV); ok = !g.rows.empty(); }
        catch (const std::exception&) { ok = false; } }
    return ok ? &g : nullptr;
}

// Hover-Basissatz fuer die synthetischen Safety-Tests: Drohne steht bei
// [0,0,1], Referenz haelt dieselbe Position, Lage level.
static void wire_hover(MCU_FLAT::ExtU_mcu_flat_T& u) {
    u.Bus_IMU_k.imu_acc[2] = 9.81;                 // Schwerkraft -> Mahony level
    u.Bus_Cmd_flat_l.q_ext[0] = 1.0;               // Mocap-Lage: Identitaet
    u.Bus_Cmd_flat_l.mocap_pos[2] = 1.0;           // Ist: 1 m Hoehe
    u.Bus_Cmd_flat_l.p_ref[2] = 1.0;               // Soll: halten
    u.Bus_Cmd_flat_l.estop = 0;
    u.Bus_Cmd_flat_l.ack = false;
    u.batt_count = 944.0;                          // ~15.74 V -> Batterie NORMAL
    u.btn_ack = false;
}

TEST(McuFlatGolden, RotorCmdMatchesGolden) {
    const NamedCsv* g = golden();
    if (!g) GTEST_SKIP() << "Golden fehlt: " << GOLDEN_MCU_FLAT_CSV
                         << " — erst log_mcu_flat_golden.m in MATLAB laufen lassen.";
    MCU_FLAT obj; obj.initialize();
    double worst = 0.0, worst_inv = 0.0; std::size_t worst_row = 0;
    for (std::size_t r = 0; r < g->rows.size(); ++r) {
        MCU_FLAT::ExtU_mcu_flat_T u{}; wire_inputs(u, *g, r);
        obj.setExternalInputs(&u); obj.step();
        const auto& y = obj.getExternalOutputs();
        double d = diff_rotor(y, *g, r);
        if (d > worst) { worst = d; worst_row = r; }
        ASSERT_LE(d, (double)GOLDEN_TOL)
            << "rotor_cmd-Divergenz in Zeile " << r
            << " (t=" << g->get(r,"t") << " s), |dq|=" << d;
        ASSERT_EQ(diff_led(y, *g, r), 0.0)
            << "led-Divergenz in Zeile " << r
            << ": got " << (int)y.led << " expected " << g->get(r,"led.1");
        ASSERT_LE(diff_throttle(y, *g, r), (double)GOLDEN_TOL)
            << "throttle-Golden-Divergenz in Zeile " << r;
        // Struktur-Invariante (wie Kaskade): gemeinsamer Spannungsfaktor
        // throttle_i/polyval(P,omega_i) fuer alle nicht geklemmten Kanaele +
        // implizierte Spannung innerhalb der Klemmgrenzen.
        {
            double ratio = 0.0; bool have = false;
            for (int i = 0; i < 4; ++i) {
                double eq = throttle_eq(y.rotor_cmd[i]);
                if (eq <= 1e-9) continue;
                if (y.throttle[i] >= mcuref::THROTTLE_MAX - 1e-9) continue;
                if (y.throttle[i] <= mcuref::THROTTLE_MIN + 1e-9) continue;
                double ri = y.throttle[i] / eq;
                if (!have) { ratio = ri; have = true; }
                double di = std::abs(ri - ratio);
                if (di > worst_inv) { worst_inv = di; }
                ASSERT_LE(di, 1e-9)
                    << "throttle-Invariante: Kanal " << i << " hat einen anderen "
                    << "Spannungsfaktor als Kanal 0, Zeile " << r;
            }
            if (have) {
                double V_impl = mcuref::U_DS / ratio;
                ASSERT_GE(V_impl, mcuref::V_THR_MIN - 1e-6)
                    << "implizierte Spannung " << V_impl << " V unter der Klemme, Zeile " << r;
                ASSERT_LE(V_impl, mcuref::V_THR_MAX + 1e-6)
                    << "implizierte Spannung " << V_impl << " V ueber der Klemme, Zeile " << r;
            }
        }
    }
    RecordProperty("worst_abs_diff", worst);
    RecordProperty("worst_row", (int)worst_row);
    RecordProperty("worst_throttle_invariant", worst_inv);
    SUCCEED() << "max|dq|=" << worst << " @ Zeile " << worst_row
              << "; max throttle-Invariante=" << worst_inv;
}

// Zwei frische Laeufe muessen bit-identisch sein (Determinismus + Reset).
TEST(McuFlatGolden, DeterministicAcrossFreshInstances) {
    const NamedCsv* g = golden();
    if (!g) GTEST_SKIP() << "Golden fehlt: " << GOLDEN_MCU_FLAT_CSV;
    auto run_once = [&](std::vector<std::array<double,9>>& out){
        MCU_FLAT obj; obj.initialize();
        for (std::size_t r = 0; r < g->rows.size(); ++r) {
            MCU_FLAT::ExtU_mcu_flat_T u{}; wire_inputs(u, *g, r);
            obj.setExternalInputs(&u); obj.step();
            const auto& y = obj.getExternalOutputs();
            out.push_back({y.rotor_cmd[0],y.rotor_cmd[1],y.rotor_cmd[2],y.rotor_cmd[3],
                           (double)y.led,
                           y.throttle[0],y.throttle[1],y.throttle[2],y.throttle[3]});
        }
    };
    std::vector<std::array<double,9>> a, b; run_once(a); run_once(b);
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t r = 0; r < a.size(); ++r)
        for (int i = 0; i < 9; ++i)
            EXPECT_DOUBLE_EQ(a[r][i], b[r][i]) << "divergenz Zeile " << r << " ch " << i;
}

// Failsafe: estop==2 -> latched -> beide Aktuator-Ausgaenge auf 0.
TEST(McuFlatFailsafe, Estop2KillsThrottleAndRotor) {
    MCU_FLAT obj; obj.initialize();
    MCU_FLAT::ExtU_mcu_flat_T u{};
    wire_hover(u);
    u.Bus_Cmd_flat_l.estop = 2;                     // Hard-Kill
    for (int k = 0; k < 20; ++k) { obj.setExternalInputs(&u); obj.step(); }
    const auto& y = obj.getExternalOutputs();
    for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(0.0, y.throttle[i])  << "throttle["  << i << "] nicht gekillt bei estop=2";
        EXPECT_EQ(0.0, y.rotor_cmd[i]) << "rotor_cmd[" << i << "] nicht gekillt bei estop=2";
    }
}

// Ohne Kill muss der Flachheitsregler im Hover Schub liefern (Sanity, macht die
// Kill-Tests beweiskraeftig).
TEST(McuFlatFailsafe, HoverProducesThrust) {
    MCU_FLAT obj; obj.initialize();
    MCU_FLAT::ExtU_mcu_flat_T u{};
    wire_hover(u);
    for (int k = 0; k < 50; ++k) { obj.setExternalInputs(&u); obj.step(); }
    const auto& y = obj.getExternalOutputs();
    double s = 0.0; for (int i = 0; i < 4; ++i) s += y.throttle[i];
    EXPECT_GT(s, 0.0) << "Hover-Setup muss ohne Kill throttle > 0 liefern";
}

// Overspeed-Latch: ||gyro||>omega_max(8.5) > debounce -> Kill; haelt ohne
// ack-Flanke; re-armt nur ueber steigende Bus_Cmd_flat.ack-Flanke; gehaltenes
// ack loescht einen frischen Trip nicht.
TEST(McuFlatOverspeed, KillHoldsAndReArmsOnBusAckEdge) {
    MCU_FLAT obj; obj.initialize();
    MCU_FLAT::ExtU_mcu_flat_T u{};
    wire_hover(u);

    // Phase 1: Overspeed -> Kill-Latch.
    u.Bus_IMU_k.imu_gyro[0] = 9.0;                  // ||.||=9 > 8.5
    for (int k = 0; k < 10; ++k) { obj.setExternalInputs(&u); obj.step(); }
    {
        const auto& y = obj.getExternalOutputs();
        for (int i = 0; i < 4; ++i) {
            EXPECT_EQ(0.0, y.rotor_cmd[i]) << "rotor_cmd[" << i << "] nicht gekillt bei Overspeed";
            EXPECT_EQ(0.0, y.throttle[i])  << "throttle["  << i << "] nicht gekillt bei Overspeed";
        }
    }

    // Phase 2a: gyro 0, keine ack-Flanke -> Latch haelt.
    u.Bus_IMU_k.imu_gyro[0] = 0.0;
    for (int k = 0; k < 10; ++k) { obj.setExternalInputs(&u); obj.step(); }
    {
        const auto& y = obj.getExternalOutputs();
        double s = 0.0; for (int i = 0; i < 4; ++i) s += y.throttle[i];
        EXPECT_EQ(0.0, s) << "Latch darf ohne ack-Flanke nicht selbst freigeben";
    }

    // Phase 2b: steigende ack-Flanke -> Re-Arm.
    u.Bus_Cmd_flat_l.ack = true;
    for (int k = 0; k < 10; ++k) { obj.setExternalInputs(&u); obj.step(); }
    {
        const auto& y = obj.getExternalOutputs();
        double s = 0.0; for (int i = 0; i < 4; ++i) s += y.throttle[i];
        EXPECT_GT(s, 0.0) << "Bus_Cmd_flat.ack-Flanke muss re-armen";
    }

    // Phase 2c: gehaltenes ack + frischer Overspeed -> muss trotzdem latchen.
    u.Bus_IMU_k.imu_gyro[0] = 9.0;
    for (int k = 0; k < 10; ++k) { obj.setExternalInputs(&u); obj.step(); }
    {
        const auto& y = obj.getExternalOutputs();
        for (int i = 0; i < 4; ++i)
            EXPECT_EQ(0.0, y.throttle[i])
                << "gehaltenes ack darf einen frischen Trip nicht loeschen";
    }
}

// Lokaler Taster: steigende btn_ack-Flanke killt; gehaltener Taster sperrt das
// Re-Armen; Taster los + neue Bus_Cmd_flat.ack-Flanke re-armt.
TEST(McuFlatButton, EdgeKillsHeldBlocksRearmBusAckClears) {
    MCU_FLAT obj; obj.initialize();
    MCU_FLAT::ExtU_mcu_flat_T u{};
    wire_hover(u);

    // Phase A: Taster-Flanke -> Kill.
    u.btn_ack = true;
    for (int k = 0; k < 10; ++k) { obj.setExternalInputs(&u); obj.step(); }
    {
        const auto& y = obj.getExternalOutputs();
        double s = 0.0; for (int i = 0; i < 4; ++i) s += y.throttle[i];
        EXPECT_EQ(0.0, s) << "Taster-Flanke muss killen";
    }

    // Phase B: Taster gehalten + ack-Flanke -> darf NICHT re-armen.
    u.Bus_Cmd_flat_l.ack = true;
    for (int k = 0; k < 10; ++k) { obj.setExternalInputs(&u); obj.step(); }
    {
        const auto& y = obj.getExternalOutputs();
        double s = 0.0; for (int i = 0; i < 4; ++i) s += y.throttle[i];
        EXPECT_EQ(0.0, s) << "gehaltener Taster muss das Re-Armen sperren";
    }

    // Phase C: Taster los, neue ack-Flanke -> re-armt.
    u.btn_ack = false;
    u.Bus_Cmd_flat_l.ack = false;
    for (int k = 0; k < 3; ++k) { obj.setExternalInputs(&u); obj.step(); }
    u.Bus_Cmd_flat_l.ack = true;
    for (int k = 0; k < 10; ++k) { obj.setExternalInputs(&u); obj.step(); }
    {
        const auto& y = obj.getExternalOutputs();
        double s = 0.0; for (int i = 0; i < 4; ++i) s += y.throttle[i];
        EXPECT_GT(s, 0.0) << "Taster los + ack-Flanke muss re-armen";
    }
}

// Tilt-Cutoff: Mahony wird ueber q_ext (kE stark) + passenden Accel in eine
// 95-deg-Kippung gezogen; nach Entprellung (80 Ticks) muss der Kill greifen.
// 95 statt 85 deg (Kaskaden-Test), weil init_safety aktuell tilt_max_deg=90
// setzt — 85 deg liegt INNERHALB der Huelle und darf nicht killen.
TEST(McuFlatTilt, SustainedTiltKills) {
    MCU_FLAT obj; obj.initialize();
    MCU_FLAT::ExtU_mcu_flat_T u{};
    wire_hover(u);
    const double d2r = 3.14159265358979323846 / 180.0;
    const double phi = 95.0 * d2r;                  // Rollwinkel um x, > tilt_max 90
    u.Bus_Cmd_flat_l.q_ext[0] = std::cos(phi / 2);  // 85 deg Roll (scalar-first)
    u.Bus_Cmd_flat_l.q_ext[1] = std::sin(phi / 2);
    u.Bus_IMU_k.imu_acc[0] = 0.0;
    u.Bus_IMU_k.imu_acc[1] = 9.81 * std::sin(phi);
    u.Bus_IMU_k.imu_acc[2] = 9.81 * std::cos(phi);

    for (int k = 0; k < 2000; ++k) { obj.setExternalInputs(&u); obj.step(); }
    const auto& y = obj.getExternalOutputs();
    for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(0.0, y.throttle[i])  << "throttle["  << i << "] nicht gekillt bei >80 deg Tilt";
        EXPECT_EQ(0.0, y.rotor_cmd[i]) << "rotor_cmd[" << i << "] nicht gekillt bei >80 deg Tilt";
    }
}

// Arming-Reset: der Regler rechnet bei aktivem Kill weiter, seine drei
// Integratorzustaende (zeta1, zeta2, eint) duerfen dabei aber NICHT weglaufen —
// sonst schlaegt beim Quittieren der aufgelaufene Zustand voll durch (auf HW
// gemessen: 1.91x Hover-Schub statt 1.00x, und ein Nickmoment von 0.39 Nm aus
// einem Integrator, der bei ep=0 nicht mehr abbaut).
//
// Test: zwei Instanzen, BEIDE 5 s unter Kill (damit Mahony und Luenberger in
// beiden gleich weit konvergiert sind — sonst misst man den Beobachter mit).
// Unterschied ist allein der Positionsfehler WAEHREND des Kills:
//   A) fuenf Sekunden mit grossem Fehler (p_ref weit weg)  -> wuerde aufintegrieren
//   B) fuenf Sekunden ohne Fehler (p_ref == mocap_pos)     -> nichts zum Laden
// Danach bekommen beide dasselbe p_ref und werden per ack-Flanke freigegeben.
// Mit Arming-Reset muessen sie identisch kommandieren; ohne war A ~1.9x hoeher.
TEST(McuFlatArmingReset, StateDoesNotWindUpDuringKill) {
    const double PREF_Z = 1.5;                      // Referenz 0.5 m ueber der Drohne
    auto thrust_sum = [](const MCU_FLAT::ExtY_mcu_flat_T& y) {
        double s = 0.0; for (int i = 0; i < 4; ++i) s += y.throttle[i]; return s;
    };
    // laeuft 5 s unter Kill mit p_ref_z = kill_ref, gibt dann bei PREF_Z frei
    auto run = [&](double kill_ref) {
        MCU_FLAT obj; obj.initialize();
        MCU_FLAT::ExtU_mcu_flat_T u{}; wire_hover(u);
        u.Bus_Cmd_flat_l.p_ref[2] = kill_ref;
        u.Bus_Cmd_flat_l.estop = 2;                 // Kill haelt
        for (int k = 0; k < 5000; ++k) { obj.setExternalInputs(&u); obj.step(); }
        EXPECT_EQ(0.0, thrust_sum(obj.getExternalOutputs())) << "Kill muss die Ausgaenge nullen";
        u.Bus_Cmd_flat_l.p_ref[2] = PREF_Z;         // ab hier identische Referenz
        u.Bus_Cmd_flat_l.estop = 0;                 // freigeben
        u.Bus_Cmd_flat_l.ack = true;                // ack-Flanke loest den Latch
        for (int k = 0; k < 20; ++k) { obj.setExternalInputs(&u); obj.step(); }
        return thrust_sum(obj.getExternalOutputs());
    };

    double thrA = run(PREF_Z);                      // grosser Fehler waehrend Kill
    double thrB = run(1.0);                         // kein Fehler waehrend Kill

    EXPECT_GT(thrB, 0.0) << "Referenzlauf muss nach dem Re-Arm Schub liefern";
    EXPECT_NEAR(thrA, thrB, 0.02 * thrB)
        << "Reglerzustand ist waehrend des Kills weggelaufen: thrA=" << thrA
        << " vs thrB=" << thrB << " (Arming-Reset wirkt nicht)";
}

// Yaw-Vorzeichen: der Regler muss den Drehsinn kennen. Frueher kam der Gierwinkel
// aus acos(eY'*yC) und war damit immer >= 0 -- bei negativem Ist-Yaw zeigte das
// Giermoment in die FALSCHE Richtung (Flug 03.08.2026: Weglauf bis -118 deg, dazu
// eine Singularitaet des dphi-Nenners bei -45 deg mit -203 deg/s Ratensprung).
// Test ohne Kenntnis der Mixer-Vorzeichen: dieselbe Auslenkung nach +30 und -30 deg
// muss betragsgleiche, ENTGEGENGESETZTE Giermomente kommandieren. Mit dem alten
// acos sind beide Faelle identisch (phi = |psi|) -> gleiches Vorzeichen -> Fail.
TEST(McuFlatYaw, TorqueIsAntisymmetricInYawError) {
    const double d2r = 3.14159265358979323846 / 180.0;
    // tau_z-Zeile des Mixers ist [-1 +1 -1 +1] -> Proxy aus den Throttles
    auto tau_z = [](const MCU_FLAT::ExtY_mcu_flat_T& y) {
        return (y.throttle[1] + y.throttle[3]) - (y.throttle[0] + y.throttle[2]);
    };
    auto run = [&](double psi_deg) {
        MCU_FLAT obj; obj.initialize();
        MCU_FLAT::ExtU_mcu_flat_T u{};
        wire_hover(u);
        const double h = psi_deg * d2r / 2.0;
        u.Bus_Cmd_flat_l.q_ext[0] = std::cos(h);   // reine Gierung um z
        u.Bus_Cmd_flat_l.q_ext[3] = std::sin(h);
        // yaw_ref bleibt 0 -> der Regler muss zurueck auf 0 drehen
        for (int k = 0; k < 3000; ++k) { obj.setExternalInputs(&u); obj.step(); }
        return tau_z(obj.getExternalOutputs());
    };
    const double tp = run(+30.0);
    const double tm = run(-30.0);
    // Schutz gegen einen vakuum-gruenen Test: es muss ueberhaupt gestellt werden.
    ASSERT_GT(std::fabs(tp), 1.0)
        << "kein nennenswertes Giermoment (tp=" << tp << ") - Test aussagelos";
    EXPECT_LT(tp * tm, 0.0)
        << "gleiches Vorzeichen bei +30 und -30 deg Gierfehler (tp=" << tp
        << ", tm=" << tm << "): der Regler kennt den Drehsinn nicht";
    EXPECT_NEAR(tp, -tm, 0.05 * std::fabs(tp))
        << "Giermoment nicht antisymmetrisch: tp=" << tp << " tm=" << tm;
}

// Batterie-FSM: fallende Spannung -> led 0(NORMAL)->1(WARN,<=14.0)->2(CRIT,<=13.4).
TEST(McuFlatBattery, RampEscalatesLed) {
    MCU_FLAT obj; obj.initialize();
    MCU_FLAT::ExtU_mcu_flat_T u{};
    wire_hover(u);
    const double k = 0.016673728813559323;          // HW-kal. (== mcu_flat_data.cpp)
    auto settle = [&](double volts, int n) -> int {
        u.batt_count = std::round(volts / k);
        for (int i = 0; i < n; ++i) { obj.setExternalInputs(&u); obj.step(); }
        return (int)obj.getExternalOutputs().led;
    };
    EXPECT_EQ(0, settle(15.5, 30000)) << "led sollte NORMAL sein bei 15.5 V";
    EXPECT_EQ(1, settle(13.7, 30000)) << "led sollte WARN sein bei 13.7 V (<=14.0, >13.4)";
    EXPECT_EQ(2, settle(13.0, 30000)) << "led sollte CRIT sein bei 13.0 V (<=13.4)";
}
