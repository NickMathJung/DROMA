// esc_calibrate.cpp — ESC-Einlernen + Motor-Test (Bench, Serial-gefuehrt).
//
// Einmaliges Anlernen der Gas-Endpunkte (OneShot125: 512..1024 count = 125..250 us),
// damit die Flug-Firmware (drone_hal) nur noch armen muss. Die Steckrichtung der 3
// Motorphasen spielt beim Einlernen keine Rolle (es werden nur die Endpunkte
// gelernt); die Drehrichtung stellt man danach ueber die Phasenreihenfolge.
//
// !!! PROPELLER AB !!!  Beim MAX-Schritt und beim Test drehen die Motoren.
//
// Ablauf (Zeichen im Serial-Monitor senden, 115200):
//   C = Kalibrieren: MAX setzen -> dann Akku anstecken, ESC-Beeps (max) abwarten
//   m = MIN setzen -> ESC-Beeps (fertig), ESC ist scharf/kalibriert
//   0 = alle Motoren waehlen | 1..4 = einzelnen Motor waehlen
//   + / - = Test-Gas +/-5% auf Auswahl (gedeckelt auf 40 %)
//   x = alle stop (min)   |   h = Hilfe
#include <Arduino.h>

static const uint8_t PIN_PWM[4] = {33, 2, 4, 3};   // M1 CCW, M2 CW, M3 CCW, M4 CW
static constexpr int ESC_MIN = 512, ESC_MAX = 1024;
static constexpr int TEST_CAP = 40;                // Test-Gas-Deckel [%]

static int g_sel = 0;                              // 0=alle, 1..4=einzeln
static int g_thr = 0;                              // Test-Gas [%]

static int count_from_pct(int pct) { return ESC_MIN + (int)lroundf(pct * 5.12f); }

static void write_one(int i, int count) { analogWrite(PIN_PWM[i], count); }
static void write_all(int count) { for (int i = 0; i < 4; ++i) write_one(i, count); }

static void apply_test() {
    for (int i = 0; i < 4; ++i)
        write_one(i, (g_sel == 0 || g_sel == i + 1) ? count_from_pct(g_thr) : ESC_MIN);
}

static void help() {
    Serial.println(F("\n=== ESC-Einlernen (PROPS AB!) ==="));
    Serial.println(F(" C = MAX setzen -> dann Akku anstecken (max-Beeps abwarten)"));
    Serial.println(F(" m = MIN setzen -> fertig-Beeps, ESC scharf"));
    Serial.println(F(" 0=alle 1..4=Motor waehlen | +/- = Test-Gas +/-5% (max 40%)"));
    Serial.println(F(" x = ALLE STOP (min) | h = Hilfe"));
    Serial.printf( " Auswahl=%s  Test-Gas=%d%%\n", g_sel ? "" : "alle", g_thr);
    if (g_sel) Serial.printf(" (Motor M%d)\n", g_sel);
}

void setup() {
    for (int i = 0; i < 4; ++i) pinMode(PIN_PWM[i], OUTPUT);
    analogWriteResolution(12);
    for (int i = 0; i < 4; ++i) analogWriteFrequency(PIN_PWM[i], 1000);
    write_all(ESC_MIN);                            // sofort sicher = min
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {}
    help();
}

void loop() {
    if (!Serial.available()) return;
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
