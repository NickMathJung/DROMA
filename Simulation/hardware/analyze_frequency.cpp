// analyze_frequency.cpp — Motor-Einzeltest wie esc_calibrate, PLUS Spannungsausgabe.
//
// Gleiche Bedienung wie esc_calibrate (ein Motor waehlen, Gas in 5-%-Schritten),
// aber zusaetzlich druckt der Sketch einmal pro Sekunde die Batteriespannung.
// Gedacht fuer die Frequenz-/Kennlinienmessung: waehrend die Handyaufnahme laeuft,
// steht in der Serial-Ausgabe die Spannung zu jedem Throttle-Punkt — so laesst sich
// die Blattfrequenz sauber einer bekannten Spannung zuordnen (das Spannungsgesetz
// throttle ~ 1/V braucht genau das).
//
// !!! PROPELLER: fuer die Schub-/Frequenzmessung DRAUF, Drohne verschraubt. !!!
//     Zum blossen ESC-Pruefen ohne Last: AB.
//
// Ausgabe je Sekunde:
//   [t= 12s] sel=M1 thr=25%  V_raw=15.28 V  V_filt=15.29 V  I_roh=137
//     V_filt = EMA(tau=0.7 s) wie safety_battery im Modell.
//     I_roh  = roher ADC-count des Stromsensors (A16), UNKALIBRIERT — nur als
//              relativer Indikator, ob der Strom mit dem Gas steigt (Akku-Sag).
//
// Tasten (Serial-Monitor, 115200):
//   C = Kalibrieren: MAX -> Akku anstecken (max-Beeps), dann 'm'
//   m = MIN -> fertig-Beeps, ESC scharf
//   0 = alle Motoren | 1..4 = einzelnen Motor waehlen
//   + / - = Test-Gas +/-5% (max TEST_CAP)   x = STOP (min)   h = Hilfe
#include <Arduino.h>
#include <math.h>

static const uint8_t PIN_PWM[4] = {33, 2, 4, 3};   // M1 CCW, M2 CW, M3 CCW, M4 CW
static constexpr uint8_t PIN_BATT_V = 41;          // A17: Spannung (wie drone_hal)
static constexpr uint8_t PIN_BATT_I = 40;          // A16: Strom (roh, unkalibriert)
static constexpr int ESC_MIN = 512, ESC_MAX = 1024;
static constexpr int TEST_CAP = 50;                // Test-Gas-Deckel [%]

// Volt-Umrechnung identisch zu drone_hal/safety_battery: k = 15.74/944 (HW-kal.).
static constexpr double BATT_K = 15.74 / 944.0;    // V/count
// EMA-Koeffizient wie im Modell: alpha = 1 - exp(-Ts/tau), Ts=10 ms, tau=0.7 s.
static constexpr double VF_DT   = 0.010;           // s (Abtastung des Filters)
static constexpr double VF_TAU  = 0.7;             // s

static int g_sel = 0;                              // 0=alle, 1..4=einzeln
static int g_thr = 0;                              // Test-Gas [%]
static double  g_vfilt = 0.0;                      // gefilterte Spannung [V]
static bool    g_vinit = false;

static int count_from_pct(int pct) { return ESC_MIN + (int)lroundf(pct * 5.12f); }
static void write_one(int i, int count) { analogWrite(PIN_PWM[i], count); }
static void write_all(int count) { for (int i = 0; i < 4; ++i) write_one(i, count); }

static void apply_test() {
    for (int i = 0; i < 4; ++i)
        write_one(i, (g_sel == 0 || g_sel == i + 1) ? count_from_pct(g_thr) : ESC_MIN);
}

static double read_v() { return analogRead(PIN_BATT_V) * BATT_K; }

static void update_vfilt() {
    double v = read_v();
    if (!g_vinit) { g_vfilt = v; g_vinit = true; return; }
    double alpha = 1.0 - exp(-VF_DT / VF_TAU);
    g_vfilt += alpha * (v - g_vfilt);
}

static void help() {
    Serial.println(F("\n=== analyze_frequency (Motortest + Spannung) ==="));
    Serial.println(F(" PROPS je nach Zweck DRAUF (Messung) oder AB (nur ESC-Check)!"));
    Serial.println(F(" C = MAX -> Akku anstecken (max-Beeps)   m = MIN -> ESC scharf"));
    Serial.printf( " 0=alle 1..4=Motor | +/- = Test-Gas +/-5%% (max %d%%)\n", TEST_CAP);
    Serial.println(F(" x = STOP (min) | h = Hilfe"));
    Serial.printf( " Auswahl=%s  Test-Gas=%d%%\n", g_sel ? "" : "alle", g_thr);
    if (g_sel) Serial.printf(" (Motor M%d)\n", g_sel);
}

static void report(uint32_t t_s) {
    int i_raw = analogRead(PIN_BATT_I);
    Serial.printf("[t=%3lus] sel=%s thr=%d%%  V_raw=%.2f V  V_filt=%.2f V  I_roh=%d\n",
                  (unsigned long)t_s,
                  g_sel ? (g_sel == 1 ? "M1" : g_sel == 2 ? "M2" : g_sel == 3 ? "M3" : "M4") : "alle",
                  g_thr, read_v(), g_vfilt, i_raw);
}

void setup() {
    for (int i = 0; i < 4; ++i) pinMode(PIN_PWM[i], OUTPUT);
    analogWriteResolution(12);
    for (int i = 0; i < 4; ++i) analogWriteFrequency(PIN_PWM[i], 1000);
    write_all(ESC_MIN);                            // sofort sicher = min
    analogReadResolution(12);                      // gleiche Aufloesung wie drone_hal
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {}
    help();
}

void loop() {
    // --- Tastenbedienung (nicht blockierend) ---
    if (Serial.available()) {
        char c = (char)Serial.read();
        switch (c) {
            case 'C': write_all(ESC_MAX); g_thr = 0;
                      Serial.println(F(">> MAX gesetzt. JETZT Akku anstecken; auf max-Beeps warten, dann 'm'.")); break;
            case 'm': write_all(ESC_MIN); g_thr = 0;
                      Serial.println(F(">> MIN gesetzt. Auf fertig-Beeps warten -> ESC kalibriert + scharf.")); break;
            case '0': g_sel = 0; apply_test(); Serial.println(F(">> Auswahl: alle")); break;
            case '1': case '2': case '3': case '4':
                      g_sel = c - '0'; apply_test(); Serial.printf(">> Auswahl: M%d\n", g_sel); break;
            case '+': g_thr = min(g_thr + 5, TEST_CAP); apply_test(); Serial.printf(">> Test-Gas=%d%%\n", g_thr); break;
            case '-': g_thr = max(g_thr - 5, 0);        apply_test(); Serial.printf(">> Test-Gas=%d%%\n", g_thr); break;
            case 'x': case ' ': g_thr = 0; write_all(ESC_MIN); Serial.println(F(">> STOP (min).")); break;
            case 'h': help(); break;
            default: break;
        }
    }

    // --- Spannungsfilter alle VF_DT, Report einmal je Sekunde ---
    static uint32_t t_filt = 0, t_rep = 0;
    uint32_t now = millis();
    if (now - t_filt >= (uint32_t)(VF_DT * 1000.0)) { t_filt = now; update_vfilt(); }
    if (now - t_rep  >= 1000)                       { t_rep  = now; report(now / 1000); }
}
