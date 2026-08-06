// drone_hal_flat.cpp — Teensy 4.1 Firmware-Mantel um MCU_FLAT::step()
// (FLATNESS-Variante). Pendant zu drone_hal.cpp; bewusst eine EIGENE Datei,
// damit die fliegende Kaskade unangetastet und jederzeit flashbar bleibt
// (--upload-drone-flight vs --upload-drone-flat-flight).
//
// Verdrahtet Sensorik und Aktorik an die MCU-Grenze
// (Bus_IMU / Bus_Cmd_flat / batt_count -> rotor_cmd/led/throttle) und taktet
// MCU_FLAT::step() bei 1 kHz. Der Codegen-Code (Klasse MCU_FLAT) bleibt unberuehrt.
//
// UNTERSCHIEDE zur Kaskaden-HAL (alles andere ist identisch):
//   - Klasse MCU_FLAT statt MCU; Bus_Cmd_flat statt Bus_Cmd an der Grenze.
//   - OTA: ZWEI 27-B-Frames pro Zyklus (mcu_flat_packet.hpp). Ein pktf::Assembler
//     paart sie ueber die gemeinsame seq und gibt erst ein koherentes Kommando
//     frei — halb-neue Saetze erreichen den Regler nie. estop/ack kommen aus
//     JEDEM Frame sofort durch (Safety wartet nicht auf das Pairing).
//   - Der Positionsregler laeuft jetzt ONBOARD: die Drohne bekommt Mocap-Pose +
//     flache Referenzen (bis Snap) statt fertiger Lage-/Schubsollwerte.
//   - Link-Diagnose zaehlt A- und B-Frames getrennt (siehe [tick]-Zeile).
//
// Festgelegte Entscheidungen:
//   - Rate: 1 kHz Basistakt (Ts_inner=1e-3), ein step() pro Tick (SingleTasking).
//   - IMU MPU-6050 @ Wire(0)=Pin18/19, 0x68 (ADO->GND), 400 kHz.
//        Gyro FS_SEL=1 (+-500 dps, 65.5 LSB/dps);  Acc AFS_SEL=1 (+-4 g, 8192 LSB/g).
//        Achsdrehung Body<-Sensor R_bs: [x_b;y_b;z_b] = [ y_s; -x_s; z_s ].
//        Gyro-Bias: 3 s Startup-Mittelung (Drohne still), dann abziehen.
//        Acc: Hebelarm roh durchreichen (die Kompensation sitzt bewusst nicht hier).
//   - Batterie: analogRead(41) = Spannung (A17, Platine umgeloetet), 12 bit, rohe
//        counts -> batt_count (Volt-Umrechnung im Modell). Strom (Pin40/A16) ist nur
//        Telemetrie.
//   - ESC: OneShot125 via analogWriteFrequency(1000)+analogWriteResolution(12):
//        count = 512 + throttle*5.12  ->  125..250 us  (throttle bereits [0,100]).
//        Beim Boot nur Arming (min halten), keine Kalibrierung. Die ESCs sind extern
//        vorkalibriert, Endpunkte 512/1024.
//   - Status-LED: led = 3-Zustands-Warn-FSM (0 NORMAL / 1 WARN / 2 CRIT), kein
//        Ladebalken. Pin5 = WARN (state>=1), Pin10 = CRIT (state==2).
//   - nRF24L01 @ SPI1 (SCK27/MOSI26/MISO1), CE14, CSN0, IRQ9. Design A:
//        Broadcast, Auto-Ack aus, 27-Byte-Payload (beide Frame-Typen gleich
//        gross -> statische Payload reicht), App-ID-Gate via BCD.
//        250 kbit/s wie die Kaskade; MUSS mit gcs_sender_flat.cpp gleich sein.
//   - Failsafe: kein gueltiges Paket seit 200 ms -> estop=2 (Hard-Kill, safety_overspeed).
//
// Noch per HW zu bestaetigen: ADO->GND-Bodge (R8) fuer 0x68; extern eingelernte
// ESC-Endpunkte; Timing-Budget im Betrieb (Serial [tick]-Report).

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>          // TMRh20 RF24; muss begin(&SPI1) unterstuetzen
#ifdef printf
#undef printf              // RF24 (Teensy) macht '#define printf Serial.printf' -> kollidiert
#endif                     //   mit unserem Serial.printf (-> Serial.Serial.printf). Neutralisieren.
#include <SD.h>            // Teensy-Core (SdFat); BUILTIN_SDCARD = SDIO 4-Bit
#include "mcu_flat.h"          // generierte Klasse MCU_FLAT (ExtU/ExtY)
#include "mcu_flat_packet.hpp" // pktf::Assembler / id_matches (single source of truth)
#include "flight_log_flat.hpp" // Blackbox-Format (single source of truth)

// ---- Betriebsart ------------------------------------------------------------
// Zwei unabhaengige Schalter, damit sich "Motoren aus" und "ich sehe was" nicht
// gegenseitig bedingen — beim Schubtest auf der Waage braucht man beides zugleich.
//
//   HAL_REPORT      Serial-Report ~10 Hz (Gyro/Acc/Batt/Link/estop/btn/throttle)
//                   plus die [boot]-Meldungen. Kostet ~0.1 ms im 1-kHz-Tick,
//                   gemessenes tickmax 464 us gegen 1000 us Budget.
//   HAL_MOTORS_MIN  Motoren hart auf ESC_MIN, esc_arm() wird uebersprungen.
//                   Die ESCs werden damit nie scharf.
//
//   HAL_MODE_BENCH  (Props ab)      : beide  -> Motoren tot, volle Telemetrie
//   HAL_MODE_THRUST (Props! S-1)    : Report -> Motoren laufen, Telemetrie bleibt
//   HAL_MODE_FLIGHT                 : keins  -> Motoren scharf, kein Report
//
// Achtung im scharfen Zustand: das Throttle-Polynom hat die Konstante 8.404, bei
// F_des = 0 und aufgehobenem Kill stehen alle vier Motoren also auf ~8.4 %, nicht
// auf 0. Die Propeller laufen an, sobald quittiert wird.
// Gewaehlt wird genau eine Betriebsart. Der Normalweg ist hal_mode.h, das
// build_sketches.sh je nach --upload-drone-* in den Sketch-Ordner schreibt.
// Bewusst KEIN -DHAL_MODE_* : die Teensy-Recipe (platform.txt) kennt
// compiler.cpp.extra_flags nicht, ein -D dort verpufft wirkungslos — und
// "Flag verpufft" heisst hier "ESCs anders scharf als gedacht".
#if defined(__has_include)
  #if __has_include("hal_mode.h")
    #include "hal_mode.h"
  #endif
#endif

// Ohne hal_mode.h gilt die sicherste Variante (Motoren tot).
#if !defined(HAL_MODE_BENCH) && !defined(HAL_MODE_THRUST) && !defined(HAL_MODE_FLIGHT)
  #define HAL_MODE_BENCH
#endif

#if defined(HAL_MODE_BENCH)
  #define HAL_REPORT
  #define HAL_MOTORS_MIN
#elif defined(HAL_MODE_THRUST)
  #define HAL_REPORT
#endif
// HAL_MODE_FLIGHT: keins von beiden -> Motoren scharf, kein Report.

// Betriebsart im Klartext ausgeben. BENCH und THRUST sehen im Serial-Monitor
// sonst identisch aus (beide mit Report) — und der Unterschied ist genau
// "Propeller drehen" vs. "drehen nicht". Deshalb steht die Art im Boot-Log UND
// in jeder Report-Zeile (mot=on/off).
#if defined(HAL_MODE_BENCH)
  #define HAL_MODE_NAME "BENCH"
#elif defined(HAL_MODE_THRUST)
  #define HAL_MODE_NAME "THRUST"
#else
  #define HAL_MODE_NAME "FLIGHT"
#endif
#ifdef HAL_MOTORS_MIN
  #define HAL_MOT_STATE "off"
#else
  #define HAL_MOT_STATE "on"
#endif

// ------------------------------ Pinbelegung (PCB Drohne_Teensy) --------------
static constexpr uint8_t PIN_PWM[4] = {33, 2, 4, 3};   // M1 CCW, M2 CW, M3 CCW, M4 CW
static constexpr uint8_t PIN_LED       = 5;            // WARN-LED  (state>=1, gelb)
static constexpr uint8_t PIN_STAT_100  = 10;           // CRIT-LED  (state==2, rot)
static constexpr uint8_t PIN_BATT_V    = 41;           // A17: SPANNUNG (Platine umgeloetet zurueck auf 41)
static constexpr uint8_t PIN_BATT_I    = 40;           // A16: STROM (Telemetrie)
static constexpr uint8_t PIN_BCD[4]    = {17, 16, 39, 38}; // BCD 1/2/4/8, INPUT_PULLUP, active-low
static constexpr uint8_t PIN_NRF_CE    = 14;
static constexpr uint8_t PIN_NRF_CSN   = 0;
static constexpr uint8_t PIN_NRF_IRQ   = 9;            // optional (hier gepollt)
static constexpr uint8_t PIN_BTN       = 21;           // Taster: active-low (INPUT_PULLUP) -> btn_ack (lokaler Kill)

// ------------------------------ Konstanten -----------------------------------
static constexpr double  G        = 9.80665;
static constexpr double  GYRO_LSB = 65.5;             // LSB/(deg/s), FS_SEL=1
static constexpr double  ACC_LSB  = 8192.0;           // LSB/g,       AFS_SEL=1
static constexpr double  DEG2RAD  = 3.14159265358979323846 / 180.0;
static constexpr uint8_t MPU_ADDR = 0x68;
static constexpr uint8_t MPU_PWR_MGMT_1 = 0x6B, MPU_GYRO_CONFIG = 0x1B,
                         MPU_ACCEL_CONFIG = 0x1C, MPU_ACCEL_XOUT_H = 0x3B;
static constexpr uint32_t LINK_TIMEOUT_MS = 200;      // Failsafe (war 100: zu eng gegen
                                                      // senderseitige Emissions-Stalls bis ~95 ms;
                                                      // Funk selbst sauber, gaps=0, maxdt 20-95 ms)
static constexpr uint32_t BIAS_MS = 3000;             // Gyro-Bias-Mittelung
static constexpr uint32_t ARM_MS  = 2000;             // Arming-Wartezeit (ESC-Piep)
static const uint64_t     NRF_BCAST_ADDR = 0xE7E7E7E7E7ULL;
static constexpr int      ESC_MIN = 512, ESC_MAX = 1024;
static constexpr uint32_t TICK_US = 1000;             // 1-kHz-Basistakt
static constexpr uint32_t TIMING_REPORT_TICKS = 1000; // Timing-Budget alle ~1 s melden

// ------------------------------ Globals --------------------------------------
static MCU_FLAT               g_mcu;
static MCU_FLAT::ExtU_mcu_flat_T g_U;        // wird jeden Tick befuellt
// Assembler haelt A- und B-Frame und gibt erst bei gleicher seq ein koherentes
// Kommando frei; g_asm.cmd ist das letzte gueltige Kommando (ZOH).
static pktf::Assembler   g_asm;
static double            g_gyro_bias[3] = {0,0,0};
static uint8_t           g_own_id = 0;
static volatile bool     g_tick = false;
static volatile uint32_t g_t_last_rx = 0;    // millis() des letzten gueltigen Pakets
static RF24              g_radio(PIN_NRF_CE, PIN_NRF_CSN);
static IntervalTimer     g_timer;
// Timing-Budget: max. Tick-Dauer (MPU-Burst + step() + IO) messen; Overruns zaehlen.
static uint32_t          g_tick_dt_max = 0;
static uint32_t          g_tick_overruns = 0;
static uint32_t          g_tick_count = 0;
// ---- Blackbox ---------------------------------------------------------------
// Zwei Ringe in RAM2 (DMAMEM), 25 s Tiefe. Waehrend des Flugs wird NICHT auf die
// SD geschrieben -- eine Karte kann fuer ihre interne Verwaltung 50-100 ms
// blockieren, und das saehe der 1-kHz-Tick. Geschrieben wird auf Kommando, mit
// stehenden Motoren. Ring statt linearem Puffer: so liegen immer die letzten 25 s
// vor, egal wie lange vor dem Start gebootet wurde.
static constexpr uint32_t LOG_FAST_CAP = 25000;   // 25 s @ 1 kHz  -> 300.0 kB
static constexpr uint32_t LOG_SLOW_CAP =  2500;   // 25 s @ 100 Hz ->  14.5 kB
DMAMEM static flog::RecFast g_log_fast_mem[LOG_FAST_CAP];
DMAMEM static flog::RecSlow g_log_slow_mem[LOG_SLOW_CAP];
static flog::Ring<flog::RecFast> g_ring_fast;
static flog::Ring<flog::RecSlow> g_ring_slow;
static bool     g_sd_ok      = false;
static uint32_t g_log_tick   = 0;      // 1-kHz-Zaehler seit Boot
// flags: Bit0 Link-Timeout, Bit1 Puffer voll (25 s erreicht), Bit2 Aufzeichnung beendet
static uint8_t  g_log_flags  = 0;
//
// EIN FLUG, EINE AUFZEICHNUNG. Der Puffer laeuft NICHT als Ring ueber, sondern
// fuellt sich einmal linear und stoppt — sonst haette die Aufzeichnung den Flug
// laengst ueberschrieben, bis man nach der Landung mit dem Kabel da ist.
//   Start : steigende ack-Flanke ODER Freigabe (estop 2 -> 0) ODER 'z' auf der Konsole
//   Stopp : estop == 2, Puffer voll, oder Motoren nach einem Flug LOG_FREEZE_MS aus
//   Dump  : automatisch beim Stopp — aber ERST wenn die Drossel wirklich null ist.
//           Bei Linkverlust springt estop mitten im Flug auf 2; ein SD-Schreibvorgang
//           von mehreren hundert ms waehrend der Failsafe-Landung waere ein
//           Regelungsproblem. Eingefroren wird sofort, geschrieben spaeter.
static constexpr uint32_t LOG_FREEZE_MS = 3000;
static bool     g_log_on       = false; // startet erst durch einen der Trigger oben
static bool     g_log_flew     = false; // es lag schon einmal Schub an
static uint32_t g_log_t_zero   = 0;     // millis() seit die Drossel null ist
static bool     g_log_dump_req = false; // Dump angefordert (ausgefuehrt in loop(), nicht im Tick)
static bool     g_log_ack_prev = false;
static uint8_t  g_log_estop_prev = 2;   // Boot: bis der Link steht, gekillt
// Tick des LETZTEN aufgezeichneten Fast-Records. Nicht g_log_tick verwenden: der
// laeuft nach dem Stopp weiter, und der Dump kann deutlich spaeter kommen (er
// wartet auf stehende Motoren) -- die Zeitachse im Header waere dann verschoben.
static uint32_t g_log_tick_last = 0;

// Link-Diagnose (BENCH): trennt "Sender emittiert nicht 100 Hz" (Windows/USB-Pacing)
// von "auf der Luft verloren" (RF, AutoAck aus). Fenster = 1 s, ausgegeben in [tick].
// Bei ZWEI Frames je Zyklus getrennt gezaehlt — so sieht man sofort, ob eine
// Haelfte systematisch schlechter ankommt (Frame B ist der laengere Weg im
// Sendezyklus und faellt bei knappem Budget zuerst aus).
//   rxA/rxB = zugestellte A- bzw. B-Frames/s (je ~100 = Link sauber)
//   pairs   = koherente Paare/s (das, was den Regler wirklich erreicht)
//   gaps    = fehlende seq/s auf Frame A (Sender hat gesendet, kam nie an)
//   maxdt   = groesster Abstand zwischen Paaren/s [ms] -> reisst das Failsafe
static uint32_t          g_rx_a    = 0;         // zugestellte A-Frames im Fenster
static uint32_t          g_rx_b    = 0;         // zugestellte B-Frames im Fenster
static uint32_t          g_rx_pairs = 0;        // koherente Paare im Fenster
static uint32_t          g_rx_gaps = 0;         // fehlende seq im Fenster (auf A)
static uint32_t          g_rx_maxdt = 0;        // groesster Paar-Abstand [ms]
static uint8_t           g_last_seq = 0;
static bool              g_seq_init = false;
static uint32_t          g_t_prev_rx = 0;

// ------------------------------ MPU-6050 -------------------------------------
static void mpu_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
static uint8_t mpu_read_reg(uint8_t reg) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.endTransmission(false);
    Wire.requestFrom((int)MPU_ADDR, 1);
    return (uint8_t)Wire.read();
}
// Montage-Offset R_mount PRO DROHNE (BCD-id). Die IMU sitzt je Drohne unterschiedlich
// schief; in Ruhe (Motoren aus, Mocap = level, q_mocap Roll/Nick < 0.4 deg) liest das Accel
// statt [0 0 g] eine leicht geneigte Richtung -> die Neigung ist MONTAGE, nicht Drohne.
// R_mount dreht die IMU-Body-Frame in die wahre (mocap-level) Body-Frame, damit der
// Accel-only-Fallback (Mocap-Ausfall) echt waagerecht statt schief haelt. R_mount ist die
// Ausrichtdrehung align(Ruhe-acc -> [0 0 g]).
//
// Messung je Drohne: die betreffende MOUNT[id] steht als Identitaet -> in BENCH flashen,
// Drohne mocap-level und still hinlegen (Motoren aus), Ruhe-acc aus dem Report mitteln,
// R_mount rechnen (scripts/motive/mount_from_acc.m) und hier eintragen. Identitaet = noch
// nicht vermessen. Ausgewaehlt wird ueber g_own_id (BCD) in setup() -> g_R_mount.
static const double MOUNT[5][3][3] = {
    // id=0 -- unbenutzt (kein BCD gesteckt) -> Identitaet
    {{1,0,0},{0,1,0},{0,0,1}},
    // id=1 -- neu vermessen 2026-08-01 (IMU-Befestigung geaendert; Neigung ~4.6 deg aus R33)
    {{ 1.0000, -0.0003,  0.0075 },
     {-0.0003,  0.9968,  0.0800 },
     {-0.0075, -0.0800,  0.9968}},
    // id=2 -- neu vermessen 2026-08-01 (IMU-Befestigung geaendert; Neigung ~0.8 deg aus R33)
    {{ 1.0000,  0.0000, -0.0021 },
     { 0.0000,  0.9999,  0.0115 },
     { 0.0021, -0.0115,  0.9999}},
    // id=3 -- NOCH NICHT VERMESSEN
    {{1,0,0},{0,1,0},{0,0,1}},
    // id=4 -- NOCH NICHT VERMESSEN
    {{1,0,0},{0,1,0},{0,0,1}},
};
static const double (*g_R_mount)[3] = MOUNT[0];   // sichere Identitaet bis setup() die BCD-id kennt

static void apply_mount(double v[3]) {
    const double (*R)[3] = g_R_mount;
    double o0 = R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2];
    double o1 = R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2];
    double o2 = R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2];
    v[0] = o0; v[1] = o1; v[2] = o2;
}

// Burst-Read 0x3B..: liefert Gyro & Acc bereits in {B} (R_bs) und SI-Einheiten.
// gyro[rad/s], acc[m/s^2]. Keine Bias-Subtraktion hier, das macht der Caller.
static void mpu_read_body(double gyro[3], double acc[3]) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(MPU_ACCEL_XOUT_H); Wire.endTransmission(false);
    Wire.requestFrom((int)MPU_ADDR, 14);
    int16_t ax = (Wire.read()<<8)|Wire.read();
    int16_t ay = (Wire.read()<<8)|Wire.read();
    int16_t az = (Wire.read()<<8)|Wire.read();
    (void)((Wire.read()<<8)|Wire.read());               // Temp verwerfen
    int16_t gx = (Wire.read()<<8)|Wire.read();
    int16_t gy = (Wire.read()<<8)|Wire.read();
    int16_t gz = (Wire.read()<<8)|Wire.read();
    // Sensor-Frame -> SI
    double gs[3] = { gx/GYRO_LSB*DEG2RAD, gy/GYRO_LSB*DEG2RAD, gz/GYRO_LSB*DEG2RAD };
    double as[3] = { ax/ACC_LSB*G,        ay/ACC_LSB*G,        az/ACC_LSB*G        };
    // R_bs: [x_b;y_b;z_b] = [ y_s; -x_s; z_s ]
    gyro[0] =  gs[1]; gyro[1] = -gs[0]; gyro[2] = gs[2];
    acc[0]  =  as[1]; acc[1]  = -as[0]; acc[2]  = as[2];
    // Feine Montage-Korrektur (nach R_bs, vor Bias/Modell) -> wahre Body-Frame.
    apply_mount(gyro);
    apply_mount(acc);
}

// ------------------------------ BCD-ID ---------------------------------------
static uint8_t read_bcd_id() {
    uint8_t id = 0;
    for (int i = 0; i < 4; ++i) id |= (uint8_t)(!digitalRead(PIN_BCD[i])) << i; // active-low
    return id;                                                                  // 0..15
}

// ------------------------------ ESC ------------------------------------------
static inline void esc_write_all(const double throttle[4]) {
    for (int i = 0; i < 4; ++i) {
        double t = throttle[i];                        // Modell clampt bereits [0,100]
        if (t < 0.0) t = 0.0; if (t > 100.0) t = 100.0;
        int c = (int)lroundf(512.0f + (float)t * 5.12f);
        if (c < ESC_MIN) c = ESC_MIN; if (c > ESC_MAX) c = ESC_MAX;
        analogWrite(PIN_PWM[i], c);
    }
}

// ------------------------------ Status-LED -----------------------------------
static void drive_leds(uint8_t state) {
    // led = Batterie-Warn-FSM (mcu_DW.state), 3 Zustaende, kein Ladebalken:
    //   0 = NORMAL, 1 = WARN (Vf<=14.0 V), 2 = CRIT (Vf<=13.4 V).
    // Mapping: Pin5 = WARN aktiv (state>=1), Pin10 = CRIT (state==2).
    digitalWrite(PIN_LED,      state >= 1 ? HIGH : LOW);   // gelb: handeln
    digitalWrite(PIN_STAT_100, state == 2 ? HIGH : LOW);   // rot: kritisch
}

// ------------------------------ nRF ------------------------------------------
// Broadcast pollen: nur Pakete mit passender ID annehmen (Design A). Jeder Frame
// geht in den Assembler; der gibt erst bei A+B mit gleicher seq ein koherentes
// Kommando frei (g_asm.cmd). estop/ack uebernimmt er aus jedem Frame sofort.
//
// WICHTIG fuer das Failsafe: g_t_last_rx wird nur von einem VOLLSTAENDIGEN Paar
// gesetzt. Kaeme dauerhaft nur eine Haelfte an, wuerde der Regler auf einem
// halben Kommandosatz stehen — dann soll das 200-ms-Failsafe greifen, nicht ein
// Einzelframe den Link "am Leben" halten.
static void nrf_poll() {
    uint8_t buf[pktf::SIZE];
    while (g_radio.available()) {
        g_radio.read(buf, pktf::SIZE);
        if (!pktf::id_matches(buf, g_own_id)) continue;  // Fremdpaket verwerfen
        bool is_b = pktf::is_frame_b(buf);
        uint8_t seq = pktf::seq_of(buf);
        if (is_b) ++g_rx_b; else ++g_rx_a;

        // seq-Luecken auf Frame A zaehlen (ein A pro Zyklus = Sender-Takt).
        if (!is_b) {
            if (g_seq_init) g_rx_gaps += (uint8_t)(seq - g_last_seq - 1);
            g_last_seq = seq; g_seq_init = true;
        }

        if (g_asm.feed(buf)) {                           // koherentes Paar
            uint32_t now = millis();
            if (g_rx_pairs) {
                uint32_t dt = now - g_t_prev_rx;
                if (dt > g_rx_maxdt) g_rx_maxdt = dt;
            }
            g_t_prev_rx = now;
            ++g_rx_pairs;
            g_t_last_rx = now;                           // nur ein Paar haelt den Link
        }
    }
}

// ------------------------------ Startup-FSM ----------------------------------
static void esc_arm() {
    // Keine Boot-Kalibrierung (kein throttle-max-Sweep, damit es mit Props sicher
    // bleibt). Die ESCs sind extern vorkalibriert, die Endpunkte muessen 512/1024
    // (=125/250 us) sein. Hier nur scharfschalten: min-Signal halten, bis die ESCs
    // armen (Piep).
    for (int i=0;i<4;++i) analogWrite(PIN_PWM[i], ESC_MIN);
    delay(ARM_MS);
}
static void estimate_gyro_bias() {
    double g[3], a[3], sum[3] = {0,0,0}; uint32_t n = 0, t0 = millis();
    while (millis() - t0 < BIAS_MS) {                       // Drohne still halten!
        mpu_read_body(g, a);
        for (int k=0;k<3;++k) sum[k] += g[k];
        ++n; delayMicroseconds(1000);
    }
    for (int k=0;k<3;++k) g_gyro_bias[k] = (n ? sum[k]/n : 0.0);
}

// ------------------------------ 1-kHz-Tick -----------------------------------
static void on_tick() { g_tick = true; }  // ISR: nur Flag; I2C/SPI im loop()

#ifdef HAL_REPORT
// Report ~10 Hz: jeden I/O-Pfad einmal sichtbar machen. Statt F_des (Kaskade)
// zeigt die Flatness-Variante Mocap-Ist und Positions-Soll — daran sieht man am
#if defined(HAL_MODE_THRUST)
// ---------------------------- S-1-Handschub -----------------------------------
// Die Drohne kann fuer den Standlauf NICHT festgezurrt werden (Kabelbinder
// erzeugen die bekannten 2-5-Hz-Gestellschwingungen). Stattdessen: Drossel von
// Hand bis an die Abhebegrenze, kommandiert aus bench_flat ueber Funk.
// Traeger ist Bus_Cmd_flat.yaw_ref[2] (ddyaw) -- in Segment-Yaw-Trajektorien
// konstant 0, und bench_flat speist dort VORUEBERGEHEND einen Slider ein.
// NUR dieser THRUST-Build deutet das Feld als Drossel in Prozent; im
// FLIGHT-Build bleibt ddyaw ddyaw. Den Slider deshalb vor dem naechsten Flug
// wieder aus bench_flat entfernen.
// Das Modell laeuft normal weiter (k_hat, dbg, Blackbox), nur die Motoren
// hoeren im THRUST-Build auf die Hand statt auf den Regler.
static double g_thr_man = 0.0;                 // Zustand des Slew-Limiters [%]
static constexpr double THR_MAN_MAX  = 70.0;   // Schwebe lag bei 53..63 % -> Luft, aber Deckel
static constexpr double THR_MAN_SLEW = 25e-3;  // 25 %/s beim 1-kHz-Tick (Slider-Spruenge)
static void thrust_manual(double out[4], double cmd_pct, uint8_t es) {
    if (es != 0) {                             // Kill/Soft-Land/Linkverlust: SOFORT null,
        g_thr_man = 0.0;                       // nicht ueber die Rampe ausrollen
    } else {
        double tgt = cmd_pct;
        if (tgt < 0.0) tgt = 0.0;
        if (tgt > THR_MAN_MAX) tgt = THR_MAN_MAX;
        double d = tgt - g_thr_man;
        if (d >  THR_MAN_SLEW) d =  THR_MAN_SLEW;
        if (d < -THR_MAN_SLEW) d = -THR_MAN_SLEW;
        g_thr_man += d;
    }
    for (int i = 0; i < 4; ++i) out[i] = g_thr_man;
}
#endif

// Boden sofort, ob Mocap-Stream und Trajektorie plausibel ankommen.
static void selftest_report(const MCU_FLAT::ExtY_mcu_flat_T& y) {
    static uint32_t n = 0;
    if (++n < 100) return; n = 0;
    double V = g_U.batt_count * 0.016673728813559323;        // Volt wie Modell (k HW-kal. 15.74/944)
    Serial.printf("id=%u gyro[% .3f % .3f % .3f] acc[% .2f % .2f % .2f] "
                  "batt=%.0f(%.2fV) bias[% .3f % .3f % .3f] link=%lums "
                  "moc[% .2f % .2f % .2f] pref[% .2f % .2f % .2f] estop=%u btn=%u "
                  "thr[%.0f %.0f %.0f %.0f] mot=" HAL_MOT_STATE " tickmax=%luus\n",
        g_own_id,
        g_U.Bus_IMU_k.imu_gyro[0], g_U.Bus_IMU_k.imu_gyro[1], g_U.Bus_IMU_k.imu_gyro[2],
        g_U.Bus_IMU_k.imu_acc[0],  g_U.Bus_IMU_k.imu_acc[1],  g_U.Bus_IMU_k.imu_acc[2],
        g_U.batt_count, V,
        g_gyro_bias[0], g_gyro_bias[1], g_gyro_bias[2],
        (unsigned long)(millis() - g_t_last_rx),
        g_U.Bus_Cmd_flat_l.mocap_pos[0], g_U.Bus_Cmd_flat_l.mocap_pos[1], g_U.Bus_Cmd_flat_l.mocap_pos[2],
        g_U.Bus_Cmd_flat_l.p_ref[0], g_U.Bus_Cmd_flat_l.p_ref[1], g_U.Bus_Cmd_flat_l.p_ref[2],
        g_U.Bus_Cmd_flat_l.estop, (unsigned)g_U.btn_ack,
        y.throttle[0], y.throttle[1], y.throttle[2], y.throttle[3],
        (unsigned long)g_tick_dt_max);
#if defined(HAL_MODE_THRUST)
    // thr[] oben ist der MODELL-Wunsch; an den ESCs liegt der Handschub.
    Serial.printf("        S1-Handschub=%.1f%%  (Traeger ddyaw=%.2f)\n",
                  g_thr_man, g_U.Bus_Cmd_flat_l.yaw_ref[2]);
#endif
}
#endif

// ---------------------------- Blackbox-Dump ----------------------------------
#if defined(HAL_MOTORS_MIN)
static constexpr uint8_t HAL_MODE_ID = 0;   // BENCH
#elif defined(HAL_REPORT)
static constexpr uint8_t HAL_MODE_ID = 1;   // THRUST
#else
static constexpr uint8_t HAL_MODE_ID = 2;   // FLIGHT
#endif

// Einen Ring chronologisch geordnet rausschreiben: hoechstens zwei zusammen-
// haengende Stuecke (ab Lesekopf bis Ende, dann von vorn bis Schreibkopf).
// Bewusst untypisiert statt als Template: der Arduino-Praeprozessor zieht in der
// .ino automatisch Prototypen und stolpert ueber Template-Signaturen.
static void dump_ring_raw(File& f, const uint8_t* buf, uint32_t cap,
                          uint32_t head, uint32_t n, uint32_t elem) {
    if (n == 0) return;
    const uint32_t start = (n == cap) ? head : 0;
    const uint32_t first = (start + n <= cap) ? n : (cap - start);
    f.write(buf + (size_t)start * elem, (size_t)first * elem);
    if (first < n) f.write(buf, (size_t)(n - first) * elem);
}

// Puffer leeren und Aufzeichnung scharf schalten (ack-Flanke, estop-Freigabe, 'z').
static void log_restart() {
    g_ring_fast.init(g_log_fast_mem, LOG_FAST_CAP);
    g_ring_slow.init(g_log_slow_mem, LOG_SLOW_CAP);
    g_log_flags &= 0x01u;              // Voll/Beendet zuruecknehmen, Link-Bit behalten
    g_log_flew     = false;
    g_log_dump_req = false;
    g_log_on       = true;
}

// Schreibt die Puffer in eine neue Datei FLATnnn.BIN. Dauert einige hundert ms --
// der Tick wird derweil verzoegert, deshalb ist der Dump bei laufenden Motoren
// gesperrt. Der Timer laeuft weiter; g_tick ist ein Flag, es geht also hoechstens
// ein Tick verloren und danach laeuft alles normal weiter.
//
// Rueckgabe false = "jetzt nicht, spaeter nochmal" (Motoren laufen). Der Aufrufer
// haelt die Anforderung dann offen; so wird nach einem Failsafe-Kill im Flug erst
// geschrieben, wenn die Drohne wirklich steht. announce=false fuer den Auto-Dump,
// damit die Warteschleife nicht bei jedem Durchlauf meckert.
static bool blackbox_dump(bool announce) {
    if (!g_sd_ok) {
        if (announce) Serial.println("[log] keine SD-Karte erkannt");
        return true;                                   // nichts zu retten, nicht wiederholen
    }
#if defined(HAL_MODE_THRUST)
    // Im THRUST-Build treibt der Handschub die Motoren, nicht das Modell. Das
    // Modell will beim scharfen System fast immer > 0 -- gegen y.throttle
    // gesperrt kaeme der Auto-Dump also nie. Gesperrt wird gegen das, was
    // wirklich an den ESCs liegt.
    const double s = 4.0 * g_thr_man;
#else
    const MCU_FLAT::ExtY_mcu_flat_T& y = g_mcu.getExternalOutputs();
    double s = 0.0; for (int i=0;i<4;++i) s += y.throttle[i];
#endif
    if (s > 0.0) {
        if (announce) Serial.println("[log] ABGELEHNT: Motoren laufen (throttle > 0)");
        return false;                                  // spaeter nochmal
    }
    if (g_ring_fast.n == 0) {
        if (announce) Serial.println("[log] nichts aufgezeichnet");
        return true;
    }

    const bool was_on = g_log_on;
    g_log_on = false;                                  // Puffer einfrieren

    char name[16] = "FLAT000.BIN";
    for (int i = 0; i < 1000; ++i) {
        snprintf(name, sizeof(name), "FLAT%03d.BIN", i);
        if (!SD.exists(name)) break;
    }
    File f = SD.open(name, FILE_WRITE);
    if (!f) { Serial.printf("[log] SD.open(%s) fehlgeschlagen\n", name); g_log_on = was_on; return; }

    const uint32_t nf = g_ring_fast.n, ns = g_ring_slow.n;
    flog::Header h;
    flog::fill_header(h, g_own_id, HAL_MODE_ID, nf, ns,
                      (nf ? g_log_tick_last - nf + 1 : 0), g_log_tick_last, millis());
    const uint32_t t0 = millis();
    f.write((const uint8_t*)&h, sizeof(h));
    dump_ring_raw(f, (const uint8_t*)g_ring_fast.buf, g_ring_fast.cap,
                  g_ring_fast.head, g_ring_fast.n, sizeof(flog::RecFast));
    dump_ring_raw(f, (const uint8_t*)g_ring_slow.buf, g_ring_slow.cap,
                  g_ring_slow.head, g_ring_slow.n, sizeof(flog::RecSlow));
    f.close();
    Serial.printf("[log] %s: %lu fast + %lu slow = %lu B in %lu ms (flags 0x%02X)\n",
                  name, (unsigned long)nf, (unsigned long)ns,
                  (unsigned long)(sizeof(h) + nf*sizeof(flog::RecFast) + ns*sizeof(flog::RecSlow)),
                  (unsigned long)(millis()-t0), g_log_flags);
    g_log_on = was_on;
    return true;
}

// Serial-Kommandos. Bewusst minimal und nur aus loop() heraus aufgerufen.
static void blackbox_console() {
    while (Serial.available()) {
        const int ch = Serial.read();
        switch (ch) {
        case 'd': blackbox_dump(true); break;
        case 'p': if (g_log_on) { g_log_on = false; g_log_flags |= 0x04u; }
                  else          { log_restart(); }
                  Serial.printf("[log] Aufzeichnung %s\n", g_log_on ? "an" : "aus"); break;
        case 'z': log_restart();
                  Serial.println("[log] Puffer geleert, Aufzeichnung an"); break;
        case 's': Serial.printf("[log] %s | fast %lu/%lu slow %lu/%lu | tick %lu | SD %s | flags 0x%02X\n",
                                g_log_on ? "laeuft" : "beendet",
                                (unsigned long)g_ring_fast.n, (unsigned long)LOG_FAST_CAP,
                                (unsigned long)g_ring_slow.n, (unsigned long)LOG_SLOW_CAP,
                                (unsigned long)g_log_tick, g_sd_ok ? "ok" : "fehlt", g_log_flags);
                  break;
        default: break;   // Zeilenenden und Tippfehler ignorieren
        }
    }
}

// ------------------------------ setup ----------------------------------------
void setup() {
    Serial.begin(115200);                                    // Timing-Budget-Report (non-blocking)
#ifdef HAL_REPORT
    { uint32_t ts=millis(); while(!Serial && millis()-ts<2000){} }   // USB-CDC kurz abwarten
    Serial.println("[boot] 0 MODE=" HAL_MODE_NAME "  Motoren=" HAL_MOT_STATE);
    Serial.println("[boot] 1 setup start");
#endif
    for (int i=0;i<4;++i) pinMode(PIN_PWM[i], OUTPUT);
    pinMode(PIN_LED, OUTPUT); pinMode(PIN_STAT_100, OUTPUT);
    for (int i=0;i<4;++i) pinMode(PIN_BCD[i], INPUT_PULLUP);
    pinMode(PIN_BTN, INPUT_PULLUP);                          // Taster gegen GND, gedrueckt = LOW

    // ESC/OneShot125-PWM: count 512..1024 == 125..250 us
    analogWriteResolution(12);
    for (int i=0;i<4;++i) analogWriteFrequency(PIN_PWM[i], 1000);
    for (int i=0;i<4;++i) analogWrite(PIN_PWM[i], ESC_MIN);   // sofort sicher = min

    analogReadResolution(12);                                // batt_count roh 0..4095

    // MPU-6050 @ 0x68 setzt voraus: ADO->GND (PCB-Bodge, Pull-Down R8 bestueckt).
    // Ohne Bodge floatet ADO -> Adresse 0x69; dann MPU_ADDR anpassen.
    Wire.begin(); Wire.setClock(400000);                     // Fast-Mode Pflicht (1 kHz-Budget)
    mpu_write(MPU_PWR_MGMT_1, 0x00);                         // wake
    mpu_write(0x1A, 0x04);   // DLPF 21/20 Hz (war 0x03=44/42): dämpft Vibration im Lage-/Gyro-Pfad; 20 Hz = 8x ueber omega_Lage
    // ACCEL_CONFIG2 (0x1D): der Accel hat auf dieser MPU ein EIGENES DLPF --
    // 0x1A deckt nur das Gyro ab. Beleg aus dem Flug 05.08.2026: Rotorton
    // 220 Hz im Gyro bei -55 dB (gefiltert), in acc_z bei -14 dB (roh).
    // 0x04 = ~20 Hz, symmetrisch zum Gyro. Fuer die REGELUNG ist das egal
    // (Mahony: kE=25 >> ka=1, offline nachgerechnet 0.008 deg Unterschied) --
    // es macht die Blackbox-Tiefbandanalyse sauberer. Der Rotorton bleibt im
    // Log messbar (~-50 dB, wie im Gyro), die Drehzahl-Diagnose funktioniert
    // also weiterhin.
    // BEFUND (Readback 05.08.2026): WHO_AM_I=0x68, 0x1A=0x04, 0x1D=0x04 -- beide
    // Register nehmen den Wert an, der Accel bleibt trotzdem roh (Rotorton
    // -9.5 dB bei 225 Hz, Flug 4). Ein ECHTER MPU-6050 filtert mit 0x1A=0x04
    // Gyro UND Accel; dieser Chip filtert nur das Gyro => Klon mit totem
    // Accel-DLPF-Pfad (bekanntes Faelschungsmuster). KEIN Register repariert
    // das. Akzeptiert: die Regelung braucht das Filter nicht (Mahony kE=25,
    // offline 0.008 deg Unterschied), die Auswertung filtert offline, und der
    // rohe Rotorton bleibt als Drehzahl-Diagnose nuetzlich. Der Write bleibt
    // drin (harmlos, und auf einem echten 6500 wuerde er wirken).
    mpu_write(0x1D, 0x04);
    mpu_write(MPU_GYRO_CONFIG, 0x08);                        // FS_SEL=1 (+-500 dps)
    mpu_write(MPU_ACCEL_CONFIG, 0x08);                       // AFS_SEL=1 (+-4 g)
#ifdef HAL_REPORT
    Serial.println("[boot] 2 MPU konfiguriert");
    // Chip-Identitaet + Registerbestand: WHO_AM_I 0x68=MPU-6050, 0x70=MPU-6500,
    // 0x71=MPU-9250 (Klone melden oft krumme Werte). Dazu Rueckleser der eben
    // geschriebenen Filterregister -- "steht drin, wirkt aber nicht" ist von
    // "wird nicht angenommen" nur so zu unterscheiden.
    Serial.printf("[boot] 2a WHO_AM_I=0x%02X  CONFIG(0x1A)=0x%02X  ACCEL_CONFIG(0x1C)=0x%02X  ACCEL_CONFIG2(0x1D)=0x%02X\n",
                  mpu_read_reg(0x75), mpu_read_reg(0x1A), mpu_read_reg(0x1C), mpu_read_reg(0x1D));
#endif

    g_own_id = read_bcd_id();
    g_R_mount = MOUNT[(g_own_id < 5) ? g_own_id : 0];   // per-Drohne Montage-Offset (Identitaet falls unbekannt/unvermessen)

    // nRF Broadcast, Auto-Ack aus (Design A) auf SPI1 (26/1/27 = Default-SPI1-Pins).
    // Auf dem Teensy die SPI1-Pins explizit setzen und SPI1.begin() vor
    // RF24::begin(&SPI1) aufrufen, sonst haengt der erste SPI-Transfer in
    // RF24::begin (Peripherie noch nicht enabled).
// SELFTEST_SKIP_NRF ist reine Debug-Hilfe (nie per Default gesetzt). Ohne Funk
// bleibt estop = 2, der Kill haelt also — auch mit scharfen ESCs unkritisch.
#if defined(SELFTEST_SKIP_NRF)
  #ifdef HAL_REPORT
    Serial.println("[boot] 3 nRF UEBERSPRUNGEN (SELFTEST_SKIP_NRF)");
  #endif
#else
  #ifdef HAL_REPORT
    Serial.println("[boot] 3 nRF begin (SPI1 explizit)...");
  #endif
    SPI1.setMOSI(26); SPI1.setMISO(1); SPI1.setSCK(27);
    SPI1.begin();
    bool nrf_ok = g_radio.begin(&SPI1);
    g_radio.setAutoAck(false);
    g_radio.setPayloadSize(pktf::SIZE);                      // 27 B, A und B gleich gross
    g_radio.setDataRate(RF24_250KBPS);                       // ~10 dB Empfindlichkeit gegen den
                                                             // ~63%-On-Air-Verlust (S-3); bleibt
                                                             // auch bei 2 Frames/Zyklus (~46%
                                                             // Kanalauslastung bei 2 Drohnen).
                                                             // MUSS mit gcs_sender_flat.cpp gleich sein!
    g_radio.setChannel(76);                                  // == gcs_sender.cpp (GS + 3 Drohnen teilen)
    g_radio.openReadingPipe(1, NRF_BCAST_ADDR);
    g_radio.startListening();
  #ifdef HAL_REPORT
    Serial.printf("[boot] 4 nRF ok=%d chip=%d\n", (int)nrf_ok, (int)g_radio.isChipConnected());
  #else
    (void)nrf_ok;
  #endif
#endif

    // Startup-Sequenz (Drohne am Boden, still): ESC armen, Gyro-Bias
#ifndef HAL_MOTORS_MIN
    esc_arm();                                               // scharfschalten (min-Halten)
#endif                                                       // sonst: ESCs bleiben min (aus setup)
#ifdef HAL_REPORT
    Serial.println("[boot] 5 gyro bias (3 s, still halten)...");
#endif
    estimate_gyro_bias();

    // Init-Kommando = sicher (kein Schub), bis das erste koherente Paar kommt.
    g_asm = pktf::Assembler{};                               // alles 0, have_a/have_b false
    // q_ext bleibt bewusst das NULL-Quaternion: vor dem ersten Paket gibt es
    // keine gueltige Mocap-Referenz. Der Mahony faellt damit auf Accel-only
    // zurueck, statt sich auf eine vorgetaeuschte waagerechte Lage einzurasten.
    // (Identitaet hier waere ein "gueltiger" Bezug mit kE=25 — genau der Fehler,
    // den das reservierte Codewort 0 auf der Funkstrecke beseitigt.)
    //
    // mocap_pos und p_ref bleiben ebenfalls 0. Das ist unkritisch, weil estop=2
    // den Kill haelt, bis der Link steht: der Flachheitsregler rechnet zwar auf
    // einem Nullzustand, seine Ausgaenge werden aber vom Kill-Gate genullt.
    g_asm.cmd.estop = 2;                                     // bis Link steht: gekillt
    g_t_last_rx = 0;

    g_mcu.initialize();   // seit der Spannungskorrektur nicht mehr statisch:
                          // initialize() setzt den RT_Vfilt-Startwert im Objekt

    // Blackbox: Ringe scharf, SD nur anmelden (geschrieben wird erst auf Kommando).
    // Eine fehlende Karte darf den Flug NICHT verhindern -- dann faellt nur der
    // Dump aus, aufgezeichnet wird trotzdem.
    g_ring_fast.init(g_log_fast_mem, LOG_FAST_CAP);
    g_ring_slow.init(g_log_slow_mem, LOG_SLOW_CAP);
    g_sd_ok = SD.begin(BUILTIN_SDCARD);

    g_timer.begin(on_tick, TICK_US);                         // 1 kHz
#ifdef HAL_REPORT
    Serial.printf("[boot] 6 bias done [% .3f % .3f % .3f]; loop laeuft ab jetzt.\n",
                  g_gyro_bias[0], g_gyro_bias[1], g_gyro_bias[2]);
    Serial.printf("[boot] 7 blackbox %s, %lu s Tiefe (%lu kB RAM2) — d=dump s=status p=pause z=leeren\n",
                  g_sd_ok ? "SD bereit" : "OHNE SD (nur RAM)",
                  (unsigned long)(LOG_FAST_CAP / 1000),
                  (unsigned long)((sizeof(g_log_fast_mem)+sizeof(g_log_slow_mem))/1024));
#endif
}

// ------------------------------ loop -----------------------------------------
void loop() {
    nrf_poll();                                              // Pakete jederzeit annehmen
    blackbox_console();                                      // d=dump p=pause z=leeren s=status
    // Auto-Dump: bewusst hier und nicht im Tick. Die Anforderung bleibt offen,
    // solange die Motoren noch laufen (Failsafe-Kill im Flug) -- geschrieben wird
    // erst, wenn die Drohne wirklich steht.
    if (g_log_dump_req && blackbox_dump(false)) g_log_dump_req = false;
    if (!g_tick) return;
    g_tick = false;
    uint32_t t_tick0 = micros();                             // Timing-Budget: Tick-Start

    // 1) Bus_IMU: MPU lesen (bereits in {B}, SI), Gyro-Bias abziehen, Acc roh
    double gyro[3], acc[3];
    mpu_read_body(gyro, acc);
    for (int k=0;k<3;++k) g_U.Bus_IMU_k.imu_gyro[k] = gyro[k] - g_gyro_bias[k];
    for (int k=0;k<3;++k) g_U.Bus_IMU_k.imu_acc[k]  = acc[k];

    // 2) Bus_Cmd_flat: letztes koherentes Kommandopaar (ZOH); Watchdog -> Hard-Kill
    const bool link_lost = (millis() - g_t_last_rx > LINK_TIMEOUT_MS);
    if (link_lost) g_asm.cmd.estop = 2;
    g_log_flags = (uint8_t)((g_log_flags & ~0x01u) | (link_lost ? 0x01u : 0x00u));
    const pktf::CmdFlat& c = g_asm.cmd;
    for (int k=0;k<3;++k) g_U.Bus_Cmd_flat_l.mocap_pos[k] = c.mocap_pos[k];
    for (int k=0;k<4;++k) g_U.Bus_Cmd_flat_l.q_ext[k]     = c.q_ext[k];
    for (int k=0;k<3;++k) g_U.Bus_Cmd_flat_l.p_ref[k]     = c.p_ref[k];
    for (int k=0;k<3;++k) g_U.Bus_Cmd_flat_l.v_ref[k]     = c.v_ref[k];
    for (int k=0;k<3;++k) g_U.Bus_Cmd_flat_l.a_ref[k]     = c.a_ref[k];
    for (int k=0;k<3;++k) g_U.Bus_Cmd_flat_l.j_ref[k]     = c.j_ref[k];
    for (int k=0;k<3;++k) g_U.Bus_Cmd_flat_l.s_ref[k]     = c.s_ref[k];
    for (int k=0;k<3;++k) g_U.Bus_Cmd_flat_l.yaw_ref[k]   = c.yaw_ref[k];
    g_U.Bus_Cmd_flat_l.estop = c.estop;
    g_U.Bus_Cmd_flat_l.ack   = c.ack;

    // 3) batt_count: rohe 12-bit counts (die Volt-Umrechnung macht das Modell)
    g_U.batt_count = (double)analogRead(PIN_BATT_V);

    // 3b) btn_ack: Taster active-low (gedrueckt=LOW). Im Modell ist der Taster
    //     jetzt eine eigene Kill-Quelle: seine steigende Flanke latcht den
    //     safety_overspeed-Kill (Motoren 0), und solange er gehalten wird, bleibt
    //     das Re-Armen gesperrt. Geloest wird ausschliesslich ueber Bus_Cmd.ack.
    //     So drehen die Propeller beim Akkuwechsel garantiert nicht an.
    g_U.btn_ack = (digitalRead(PIN_BTN) == LOW);

    // 4) Ein step()
    g_mcu.setExternalInputs(&g_U);
    g_mcu.step();
    const MCU_FLAT::ExtY_mcu_flat_T& y = g_mcu.getExternalOutputs();

    // 5) Aktorik: throttle -> OneShot125, led-state -> LEDs
#if defined(HAL_MODE_THRUST)
    // S-1: Motoren hoeren auf die Hand (Slider via yaw_ref[2]), nicht auf den
    // Regler. thr_act ist auch das, was Flew-Erkennung und Blackbox sehen.
    double thr_act[4];
    thrust_manual(thr_act, g_U.Bus_Cmd_flat_l.yaw_ref[2],
                  (uint8_t)g_U.Bus_Cmd_flat_l.estop);
#else
    const double* thr_act = y.throttle;
#endif
#ifdef HAL_MOTORS_MIN
    for (int i=0;i<4;++i) analogWrite(PIN_PWM[i], ESC_MIN);  // Motoren sicher auf min
#else
    esc_write_all(thr_act);
#endif
    drive_leds(y.led);
#ifdef HAL_REPORT
    selftest_report(y);
#endif

    // 5b) Blackbox: zwei memcpy, sonst nichts. Bewusst INNERHALB der Zeitmessung,
    //     damit tickmax die Kosten mitzeigt (gemessen: unter 1 us).
    ++g_log_tick;
    {
        const bool    ack_now = g_U.Bus_Cmd_flat_l.ack;
        const uint8_t es      = (uint8_t)g_U.Bus_Cmd_flat_l.estop;
        // --- Start: Quittieren oder Freigabe. Beides setzt die Puffer zurueck, damit
        //     die Aufzeichnung am Anfang des Flugs beginnt und nicht mittendrin.
        if ((ack_now && !g_log_ack_prev) || (es == 0 && g_log_estop_prev != 0)) {
            log_restart();
        }
        g_log_ack_prev   = ack_now;
        g_log_estop_prev = es;

        double thr_sum = 0.0; for (int k=0;k<4;++k) thr_sum += thr_act[k];
        if (thr_sum > 0.0) { g_log_flew = true; g_log_t_zero = millis(); }

        // --- Stopp: drei Gruende, alle enden im selben Zustand (eingefroren + Dump
        //     angefordert). Welcher zuerst kommt, ist egal.
        if (g_log_on) {
            const bool full    = (g_ring_fast.n >= LOG_FAST_CAP);
            const bool killed  = (es == 2);
            const bool landed  = g_log_flew && (millis() - g_log_t_zero > LOG_FREEZE_MS);
            if (full) g_log_flags |= 0x02u;
            if (full || killed || landed) {
                g_log_on = false;
                g_log_flags |= 0x04u;
                g_log_dump_req = true;
            }
        }
    }
    if (g_log_on) {
        flog::RecFast rf;
        for (int k=0;k<3;++k) rf.gyro[k] = flog::q15(g_U.Bus_IMU_k.imu_gyro[k], flog::GYRO_SCALE);
        for (int k=0;k<3;++k) rf.acc[k]  = flog::q15(g_U.Bus_IMU_k.imu_acc[k],  flog::ACC_SCALE);
        g_ring_fast.push(rf);
        g_log_tick_last = g_log_tick;

        if ((g_log_tick % 10u) == 0u) {           // 100 Hz
            flog::RecSlow rs;
            rs.tick = g_log_tick;
            for (int k=0;k<3;++k) rs.mocap_pos[k] = (float)c.mocap_pos[k];
            for (int k=0;k<3;++k) rs.p_ref[k]     = (float)c.p_ref[k];
            for (int k=0;k<4;++k) rs.q_ext[k]     = (float)c.q_ext[k];
            for (int k=0;k<4;++k) {
                double th = thr_act[k] * 100.0;               // 0..100 % -> 0.01 % (ECHTER Motorwert)
                rs.thr[k] = (uint16_t)(th < 0 ? 0 : (th > 10000 ? 10000 : th + 0.5));
            }
            rs.batt_count = (uint16_t)g_U.batt_count;
            rs.estop = (uint8_t)g_U.Bus_Cmd_flat_l.estop;
            rs.led   = (uint8_t)y.led;
            rs.ack   = g_U.Bus_Cmd_flat_l.ack ? 1 : 0;
            rs.flags = g_log_flags;
            // dbg = [k_hat; F; aint(3); u_fb_raw(3); w_adapt] aus dem Modellausgang
            rs.k_hat = flog::q15(y.dbg[0], flog::KHAT_SCALE);
            rs.F     = flog::q15(y.dbg[1], flog::F_SCALE);
            for (int k=0;k<3;++k) rs.aint[k] = flog::q15(y.dbg[2+k], flog::AINT_SCALE);
            for (int k=0;k<3;++k) rs.ufb[k]  = flog::q15(y.dbg[5+k], flog::UFB_SCALE);
            rs.w_adapt = flog::q15(y.dbg[8], flog::W_SCALE);
            g_ring_slow.push(rs);
        }
    }

    // 6) Timing-Budget: max. Tick-Dauer + Overruns (>1 ms) messen, ~1x/s melden.
    uint32_t dt = micros() - t_tick0;
    if (dt > g_tick_dt_max) g_tick_dt_max = dt;
    if (dt > TICK_US) ++g_tick_overruns;
    if (++g_tick_count >= TIMING_REPORT_TICKS) {
        Serial.printf("[tick] max=%lu us, overruns=%lu / %lu | "
                      "rxA=%lu rxB=%lu pairs=%lu gaps=%lu (emit=%lu) maxdt=%lums\n",
                      (unsigned long)g_tick_dt_max, (unsigned long)g_tick_overruns,
                      (unsigned long)g_tick_count,
                      (unsigned long)g_rx_a, (unsigned long)g_rx_b,
                      (unsigned long)g_rx_pairs, (unsigned long)g_rx_gaps,
                      (unsigned long)(g_rx_a + g_rx_gaps), (unsigned long)g_rx_maxdt);
        g_tick_dt_max = 0; g_tick_overruns = 0; g_tick_count = 0;
        g_rx_a = 0; g_rx_b = 0; g_rx_pairs = 0; g_rx_gaps = 0; g_rx_maxdt = 0;
    }
}
