// drone_hal.cpp — Teensy 4.1 Firmware-Mantel um den generierten MCU::step().
// Baustein-Skelett (Teensyduino / PlatformIO). Verdrahtet Sensorik und Aktorik
// an die MCU-Grenze (Bus_IMU / Bus_Cmd / batt_count -> rotor_cmd/led/throttle)
// und taktet MCU::step() bei 1 kHz. Der Codegen-Code (Klasse MCU) bleibt unberuehrt.
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
//        Broadcast, Auto-Ack aus, 29-Byte-Payload, App-ID-Gate via BCD.
//        begin(&SPI1) (Fallback fuer aeltere Lib im Code auskommentiert).
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
#include "mcu.h"           // generierte Klasse MCU (ExtU/ExtY)
#include "mcu_packet.hpp"  // pkt::unpack / id_matches (single source of truth)

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
static MCU               g_mcu;
static MCU::ExtU_mcu_T   g_U;                // wird jeden Tick befuellt
static pkt::Cmd       g_cmd;              // letztes gueltiges Kommando (ZOH)
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
// Link-Diagnose (BENCH): trennt "Sender emittiert nicht 100 Hz" (Windows/USB-Pacing)
// von "auf der Luft verloren" (RF, AutoAck aus). Fenster = 1 s, ausgegeben in [tick].
//   rx     = zugestellte Frames/s (effektive Linkrate)
//   gaps   = fehlende seq/s      (Sender hat gesendet, kam nie an -> RF-Verlust)
//   rx+gaps= vom Sender wirklich emittierte Frames/s (~100 = Simulink ok; ~64 = OS-Takt)
//   maxdt  = groesster Ankunftsabstand/s [ms] -> reisst das 100-ms-Failsafe
static uint32_t          g_rx_win  = 0;         // zugestellte Pakete im Fenster
static uint32_t          g_rx_gaps = 0;         // fehlende seq im Fenster
static uint32_t          g_rx_maxdt = 0;        // groesster Arrival-Abstand [ms]
static uint8_t           g_last_seq = 0;
static bool              g_seq_init = false;
static uint32_t          g_t_prev_rx = 0;

// ------------------------------ MPU-6050 -------------------------------------
static void mpu_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
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
    // id=1 -- NOCH NICHT VERMESSEN (Ruhe-acc ablesen, dann Identitaet ersetzen)
    {{1,0,0},{0,1,0},{0,0,1}},
    // id=2 -- gemessen 2026-07-26 (Neigung 2.27 deg, Achse [-0.988 -0.153 0])
    {{ 0.999981676,  0.000118626, -0.006052623},
     { 0.000118626,  0.999232053,  0.039182770},
     { 0.006052623, -0.039182770,  0.999213729}},
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
// Broadcast pollen: nur Pakete mit passender ID annehmen (Design A).
static void nrf_poll() {
    uint8_t buf[pkt::SIZE];
    while (g_radio.available()) {
        g_radio.read(buf, pkt::SIZE);
        if (!pkt::id_matches(buf, g_own_id)) continue; // Fremdpaket verwerfen
        uint8_t id, seq;
        pkt::unpack(buf, g_cmd, id, seq);              // ZOH: g_cmd haelt bis zum naechsten
        uint32_t now = millis();
        if (g_seq_init) {
            g_rx_gaps += (uint8_t)(seq - g_last_seq - 1);   // fehlende Frames (mod 256)
            uint32_t dt = now - g_t_prev_rx;
            if (dt > g_rx_maxdt) g_rx_maxdt = dt;
        }
        g_last_seq = seq; g_seq_init = true; g_t_prev_rx = now;
        ++g_rx_win;
        g_t_last_rx = now;
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
// Report ~10 Hz: jeden I/O-Pfad einmal sichtbar machen.
static void selftest_report(const MCU::ExtY_mcu_T& y) {
    static uint32_t n = 0;
    if (++n < 100) return; n = 0;
    double V = g_U.batt_count * 0.016673728813559323;        // Volt wie Modell (k HW-kal. 15.74/944)
    Serial.printf("id=%u gyro[% .3f % .3f % .3f] acc[% .2f % .2f % .2f] "
                  "batt=%.0f(%.2fV) bias[% .3f % .3f % .3f] link=%lums Fdes=%.2f estop=%u btn=%u "
                  "thr[%.0f %.0f %.0f %.0f] mot=" HAL_MOT_STATE " tickmax=%luus\n",
        g_own_id,
        g_U.Bus_IMU_k.imu_gyro[0], g_U.Bus_IMU_k.imu_gyro[1], g_U.Bus_IMU_k.imu_gyro[2],
        g_U.Bus_IMU_k.imu_acc[0],  g_U.Bus_IMU_k.imu_acc[1],  g_U.Bus_IMU_k.imu_acc[2],
        g_U.batt_count, V,
        g_gyro_bias[0], g_gyro_bias[1], g_gyro_bias[2],
        (unsigned long)(millis() - g_t_last_rx), g_U.Bus_Cmd_l.F_des, g_U.Bus_Cmd_l.estop, (unsigned)g_U.btn_ack,
        y.throttle[0], y.throttle[1], y.throttle[2], y.throttle[3],
        (unsigned long)g_tick_dt_max);
}
#endif

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
    mpu_write(MPU_GYRO_CONFIG, 0x08);                        // FS_SEL=1 (+-500 dps)
    mpu_write(MPU_ACCEL_CONFIG, 0x08);                       // AFS_SEL=1 (+-4 g)
#ifdef HAL_REPORT
    Serial.println("[boot] 2 MPU konfiguriert");
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
    g_radio.setPayloadSize(pkt::SIZE);
    g_radio.setDataRate(RF24_250KBPS);                       // war 1MBPS: ~10 dB Empfindlichkeit
                                                             // gegen ~63%-On-Air-Verlust (S-3).
                                                             // MUSS mit gcs_sender.cpp gleich sein!
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

    // Init-Kommando = sicher (kein Schub), bis das erste Paket kommt
    g_cmd = pkt::Cmd{};                                   // alles 0
    g_cmd.q_des[0]=1; g_cmd.q_ref[0]=1;                   // Sollwerte: Identitaet
    // q_ext bleibt bewusst das NULL-Quaternion: vor dem ersten Paket gibt es
    // keine gueltige Mocap-Referenz. Der Mahony faellt damit auf Accel-only
    // zurueck, statt sich auf eine vorgetaeuschte waagerechte Lage einzurasten.
    // (Identitaet hier waere ein "gueltiger" Bezug mit kE=25 — genau der Fehler,
    // den das reservierte Codewort 0 auf der Funkstrecke beseitigt.)
    g_cmd.estop = 2;                                         // bis Link steht: gekillt
    g_t_last_rx = 0;

    g_mcu.initialize();   // seit der Spannungskorrektur nicht mehr statisch:
                          // initialize() setzt den RT_Vfilt-Startwert im Objekt
    g_timer.begin(on_tick, TICK_US);                         // 1 kHz
#ifdef HAL_REPORT
    Serial.printf("[boot] 6 bias done [% .3f % .3f % .3f]; loop laeuft ab jetzt.\n",
                  g_gyro_bias[0], g_gyro_bias[1], g_gyro_bias[2]);
#endif
}

// ------------------------------ loop -----------------------------------------
void loop() {
    nrf_poll();                                              // Pakete jederzeit annehmen
    if (!g_tick) return;
    g_tick = false;
    uint32_t t_tick0 = micros();                             // Timing-Budget: Tick-Start

    // 1) Bus_IMU: MPU lesen (bereits in {B}, SI), Gyro-Bias abziehen, Acc roh
    double gyro[3], acc[3];
    mpu_read_body(gyro, acc);
    for (int k=0;k<3;++k) g_U.Bus_IMU_k.imu_gyro[k] = gyro[k] - g_gyro_bias[k];
    for (int k=0;k<3;++k) g_U.Bus_IMU_k.imu_acc[k]  = acc[k];

    // 2) Bus_Cmd: letztes gueltiges Paket (ZOH); Watchdog -> Hard-Kill
    if (millis() - g_t_last_rx > LINK_TIMEOUT_MS) g_cmd.estop = 2;
    g_U.Bus_Cmd_l.F_des = g_cmd.F_des;
    for (int k=0;k<4;++k) g_U.Bus_Cmd_l.q_des[k] = g_cmd.q_des[k];
    for (int k=0;k<4;++k) g_U.Bus_Cmd_l.q_ref[k] = g_cmd.q_ref[k];
    for (int k=0;k<3;++k) g_U.Bus_Cmd_l.Omega_ref[k] = g_cmd.Omega_ref[k];
    for (int k=0;k<3;++k) g_U.Bus_Cmd_l.tau_ref[k] = g_cmd.tau_ref[k];
    for (int k=0;k<4;++k) g_U.Bus_Cmd_l.q_ext[k] = g_cmd.q_ext[k];
    g_U.Bus_Cmd_l.estop = g_cmd.estop;
    g_U.Bus_Cmd_l.ack   = g_cmd.ack;

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
    const MCU::ExtY_mcu_T& y = g_mcu.getExternalOutputs();

    // 5) Aktorik: throttle -> OneShot125, led-state -> LEDs
#ifdef HAL_MOTORS_MIN
    for (int i=0;i<4;++i) analogWrite(PIN_PWM[i], ESC_MIN);  // Motoren sicher auf min
#else
    esc_write_all(y.throttle);
#endif
    drive_leds(y.led);
#ifdef HAL_REPORT
    selftest_report(y);
#endif

    // 6) Timing-Budget: max. Tick-Dauer + Overruns (>1 ms) messen, ~1x/s melden.
    uint32_t dt = micros() - t_tick0;
    if (dt > g_tick_dt_max) g_tick_dt_max = dt;
    if (dt > TICK_US) ++g_tick_overruns;
    if (++g_tick_count >= TIMING_REPORT_TICKS) {
        Serial.printf("[tick] max=%lu us, overruns=%lu / %lu | "
                      "rx=%lu gaps=%lu (emit=%lu) maxdt=%lums\n",
                      (unsigned long)g_tick_dt_max, (unsigned long)g_tick_overruns,
                      (unsigned long)g_tick_count,
                      (unsigned long)g_rx_win, (unsigned long)g_rx_gaps,
                      (unsigned long)(g_rx_win + g_rx_gaps), (unsigned long)g_rx_maxdt);
        g_tick_dt_max = 0; g_tick_overruns = 0; g_tick_count = 0;
        g_rx_win = 0; g_rx_gaps = 0; g_rx_maxdt = 0;
    }
}
