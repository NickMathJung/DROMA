// battery_health.cpp — Akku-Innenwiderstand per Lasttest (Teensy, Serial-gefuehrt).
//
// Idee: R_i = (V_leer - V_last) / I_last. Der Strom wird NICHT gemessen (der
// PM06-Stromsensor ist bei Bench-Stroemen im Rauschen), sondern aus dem GEMFAN-
// Datenblatt + der aktuellen Spannung GERECHNET. Das ist zulaessig, weil die
// Drehzahl-Spannungs-Beziehung omega ~ (duty*U) auf HW belegt ist (Frequenz-
// messung, 3 Spannungsebenen): bei Throttle thr und Spannung V verhaelt sich der
// Motor wie das Datenblatt bei teq = thr*V/22.2, und gleiche Drehzahl = gleiche
// mechanische Leistung => I = I_ds(teq) * 22.2/V (konstante Leistung).
//
// !!! PROPELLER MUESSEN DRAUF SEIN !!!  Ohne Last kein Strom, kein Sag, kein
//     Messwert. Drohne FEST verschrauben — bei 30 % ziehen 4 Motoren ~10 A und
//     erzeugen ~560 g Schub (hebt die 985-g-Drohne nicht, kann sie aber kippen).
//
// Ablauf ('t' im Serial-Monitor, 115200):
//   Phase 1 (4 s)  alle Motoren min  -> V_leer
//   Phase 2 (12 s) alle Motoren 30 % -> V_last (nach Einschwingen) + Enddrift
//   Phase 3 (4 s)  alle Motoren min  -> Erholung
// Danach Urteil: R_i [mOhm], erwarteter Hover-Sag, Stromsensor-Kalibrierung.
#include <Arduino.h>

static const uint8_t PIN_PWM[4] = {33, 2, 4, 3};
static constexpr uint8_t PIN_BATT_V = 41;          // A17 Spannung
static constexpr uint8_t PIN_BATT_I = 40;          // A16 Strom (roh)
static constexpr int ESC_MIN = 512, ESC_MAX = 1024;
static constexpr int   TEST_THR   = 30;            // % (4 Motoren ~10 A; Hover ~13 A @35 %)
static constexpr double BATT_K    = 15.74 / 944.0; // V/count (HW-kal., wie drone_hal)
static constexpr double U_DS      = 22.2;          // V, Datenblattspannung
static constexpr double I_HOVER   = 13.0;          // A, ~Hover-Gesamtstrom (fuer die Prognose)

// GEMFAN 51499 @ 22.2 V: Throttle[%] -> Strom[A] pro Motor.
static const double DS_THR[11] = {0,10,20,30,40,50,60,70,80,90,100};
static const double DS_I  [11] = {0,0.44,1.68,3.55,6.36,10.50,14.95,20.01,27.35,35.50,42.20};

static int count_from_pct(int p) { return ESC_MIN + (int)lroundf(p * 5.12f); }
static void write_all(int c) { for (int i=0;i<4;++i) analogWrite(PIN_PWM[i], c); }
static double read_v() { return analogRead(PIN_BATT_V) * BATT_K; }

// lineare Interpolation Datenblattstrom (1 Motor, 22.2 V)
static double ds_current(double teq) {
    if (teq <= 0) return 0;
    if (teq >= 100) return DS_I[10];
    for (int i=1;i<11;++i)
        if (teq <= DS_THR[i]) {
            double f = (teq - DS_THR[i-1]) / (DS_THR[i] - DS_THR[i-1]);
            return DS_I[i-1] + f * (DS_I[i] - DS_I[i-1]);
        }
    return DS_I[10];
}
// Gesamtstrom 4 Motoren bei Throttle thr, Spannung V (konstante Leistung)
static double i_total(double thr, double V) {
    if (V < 1.0) return 0;
    return 4.0 * ds_current(thr * V / U_DS) * U_DS / V;
}

// --- Zeitfenster [ms] der automatischen Sequenz ---
static constexpr uint32_t T_OC0=2000, T_OC1=4000;       // V_leer mitteln
static constexpr uint32_t T_RAMP=5000;                  // bis hier auf Last hochfahren
static constexpr uint32_t T_RI0=7000, T_RI1=9000;       // V_last (eingeschwungen)
static constexpr uint32_t T_CUR0=6000, T_CUR1=16000;    // Strom-Rohwert mitteln
static constexpr uint32_t T_END0=15000, T_END1=17000;   // Enddrift
static constexpr uint32_t T_LOAD_OFF=17000;             // Last aus
static constexpr uint32_t T_REC0=20000, T_REC1=22000;   // Erholung mitteln
static constexpr uint32_t T_DONE=22000;

static bool     g_run = false;
static uint32_t g_t0  = 0;
struct Acc { double sum=0; uint32_t n=0; void add(double x){sum+=x;++n;} double avg()const{return n?sum/n:0;} void reset(){sum=0;n=0;} };
static Acc a_oc, a_ri, a_end, a_rec, a_cur;

static void banner() {
    Serial.println(F("\n=== battery_health — Akku-Lasttest ==="));
    Serial.println(F(" !!! PROPELLER DRAUF, Drohne FEST verschraubt !!!"));
    Serial.printf( " Test: 4 Motoren %d %% fuer 12 s. 't' startet, 'x' bricht ab.\n", TEST_THR);
}

void setup() {
    for (int i=0;i<4;++i) pinMode(PIN_PWM[i], OUTPUT);
    analogWriteResolution(12);
    for (int i=0;i<4;++i) analogWriteFrequency(PIN_PWM[i], 1000);
    write_all(ESC_MIN);                 // sofort sicher + ESC-Arming (min ~2 s halten)
    analogReadResolution(12);
    Serial.begin(115200);
    uint32_t t=millis(); while(!Serial && millis()-t<3000){}
    delay(2000);                        // ESC-Arming abwarten
    banner();
}

static void finish() {
    write_all(ESC_MIN); g_run = false;
    double Voc=a_oc.avg(), Vld=a_ri.avg(), Vend=a_end.avg(), Vrec=a_rec.avg();
    double I  = i_total(TEST_THR, Vld);
    double Ri = (I>0.1) ? 1000.0*(Voc - Vld)/I : -1;   // mOhm
    double drift = Vld - Vend;
    double hover_sag = (Ri>0) ? Ri/1000.0 * I_HOVER : -1;
    double cur_scale = (I>0.1) ? a_cur.avg()/I : 0;    // counts/A (grobe Sensor-Kal.)

    Serial.println(F("\n---------- ERGEBNIS ----------"));
    Serial.printf(" V_leer      : %.2f V\n", Voc);
    Serial.printf(" V_last (%d%%): %.2f V   (Sag %.2f V)\n", TEST_THR, Vld, Voc-Vld);
    Serial.printf(" I_last (gsch): %.1f A   (aus Datenblatt @ %.1f V)\n", I, Vld);
    Serial.printf(" Enddrift 12s: %.2f V   (%s)\n", drift,
                  drift>0.4 ? "hoch: Pack sackt unter Dauerlast" : "ok");
    Serial.printf(" Erholung    : %.2f V   (%s)\n", Vrec,
                  (Voc-Vrec)>0.3 ? "traege: entladen/gealtert" : "ok");
    if (Ri < 0) { Serial.println(F(" R_i: nicht berechenbar (kein Strom - Props ab?)")); }
    else {
        const char* verdict = Ri<30 ? "GESUND" : Ri<70 ? "GRENZWERTIG" : "DEFEKT - NICHT FLIEGEN";
        Serial.printf(" >> R_i (Pack): %.0f mOhm  -> %s\n", Ri, verdict);
        Serial.printf("    (gesund <30, grenzwertig 30-70, defekt >70 mOhm)\n");
        Serial.printf(" >> Prognose Hover (~%.0f A): Sag %.2f V -> unter Last ~%.2f V\n",
                      I_HOVER, hover_sag, Voc - hover_sag);
        if (Voc - hover_sag < 13.5)
            Serial.println(F("    ACHTUNG: Hover-Spannung nahe/unter 13.5 V — Flugmarge knapp."));
    }
    Serial.printf(" Stromsensor : I_roh mittel %.0f counts ~ %.1f counts/A (grob)\n",
                  a_cur.avg(), cur_scale);
    Serial.println(F("------------------------------"));
    Serial.println(F(" 't' fuer neuen Lauf."));
}

void loop() {
    if (Serial.available()) {
        char c = (char)Serial.read();
        if (c=='t' && !g_run) {
            g_run=true; g_t0=millis();
            a_oc.reset();a_ri.reset();a_end.reset();a_rec.reset();a_cur.reset();
            Serial.println(F(">> Start. Phase 1: Leerlauf..."));
        } else if (c=='x') {
            write_all(ESC_MIN); g_run=false; Serial.println(F(">> Abgebrochen (min)."));
        } else if (c=='h') banner();
    }
    if (!g_run) return;

    static uint32_t last=0; uint32_t now=millis(); uint32_t e=now-g_t0;
    if (now-last < 20) return;                 // ~50 Hz Abtastung
    last=now;
    double v=read_v();

    // Motoransteuerung je Phase
    if      (e < T_RAMP)     write_all(ESC_MIN + (int)((count_from_pct(TEST_THR)-ESC_MIN) *
                                     (e>4000 ? (double)(e-4000)/(T_RAMP-4000) : 0.0)));  // Rampe 4->5s
    else if (e < T_LOAD_OFF) write_all(count_from_pct(TEST_THR));
    else                     write_all(ESC_MIN);

    // Messfenster
    if (e>=T_OC0  && e<T_OC1)  a_oc.add(v);
    if (e>=T_RI0  && e<T_RI1)  a_ri.add(v);
    if (e>=T_END0 && e<T_END1) a_end.add(v);
    if (e>=T_REC0 && e<T_REC1) a_rec.add(v);
    if (e>=T_CUR0 && e<T_CUR1) a_cur.add((double)analogRead(PIN_BATT_I));

    // Live-Fortschritt 1x/s
    static uint32_t lastrep=0;
    if (now-lastrep>=1000) { lastrep=now;
        Serial.printf("[%2lus] V=%.2f\n",(unsigned long)(e/1000),v);
    }
    if (e>=T_DONE) finish();
}
