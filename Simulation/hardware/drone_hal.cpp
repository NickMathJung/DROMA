// drone_hal.cpp — Teensy 4.1 Firmware-Mantel um den generierten MCU::step().
// Baustein-Skelett (Teensyduino / PlatformIO). Verdrahtet die Sensorik/Aktorik
// an die MCU-Grenze (Bus_IMU / Bus_Cmd / batt_count -> rotor_cmd/led/throttle),
// taktet MCU::step() bei 1 kHz. Codegen-Code (Klasse MCU) bleibt unberuehrt.
//
// GELOCKTE ENTSCHEIDUNGEN (Handover Teil 6 + Step-4-Session):
//   - Rate: 1 kHz Basistakt (Ts_inner=1e-3). Ein step() pro Tick (SingleTasking).
//   - IMU MPU-6050 @ Wire(0)=Pin18/19, 0x68 (ADO->GND), 400 kHz.
//        Gyro FS_SEL=1 (+-500 dps, 65.5 LSB/dps);  Acc AFS_SEL=1 (+-4 g, 8192 LSB/g).
//        Achsdrehung Body<-Sensor R_bs: [x_b;y_b;z_b] = [ y_s; -x_s; z_s ].
//        Gyro-Bias: 3 s Startup-Mittelung (Drohne still) -> abziehen.
//        Acc: hebelarm-ROH durchreichen (Kompensation sitzt bewusst NICHT hier).
//   - Batterie: analogRead(41)=SPANNUNG (A17), 12 bit, ROHE counts -> batt_count
//        (Volt-Umrechnung macht das Modell, S6). Strom (Pin40/A16) = nur Telemetrie.
//   - ESC: OneShot125 via analogWriteFrequency(1000)+analogWriteResolution(12):
//        count = 512 + throttle*5.12  ->  125..250 us  (throttle bereits [0,100]).
//   - nRF24L01 @ SPI1 (SCK27/MOSI26/MISO1), CE14, CSN0, IRQ9. Design A:
//        Broadcast, Auto-Ack AUS, 29-Byte-Payload, App-ID-Gate via BCD.
//   - Failsafe: kein gueltiges Paket seit 100 ms -> estop=2 (Hard-Kill, safety_overspeed).
//
// OFFEN (mit TODO markiert): Status-LED-Pins 25/50/75 %, ESC-Einlern-Timings.

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>          // TMRh20 RF24; muss begin(&SPI1) unterstuetzen
#include "mcu.h"           // generierte Klasse MCU (ExtU/ExtY)
#include "mcu_packet.hpp"  // pkt::unpack / id_matches (single source of truth)

// ------------------------------ Pinbelegung (PCB Drohne_Teensy) --------------
static constexpr uint8_t PIN_PWM[4] = {33, 2, 4, 3};   // M1 CCW, M2 CW, M3 CCW, M4 CW
static constexpr uint8_t PIN_LED       = 5;            // Sammel-LED
static constexpr uint8_t PIN_STAT_100  = 10;           // STATUS_100%
// TODO: STATUS_25/50/75 %-Pins nachtragen, sobald aus dem Schaltplan bestaetigt.
static constexpr uint8_t PIN_BATT_V    = 41;           // A17 SPANNUNG
static constexpr uint8_t PIN_BATT_I    = 40;           // A16 STROM (Telemetrie)
static constexpr uint8_t PIN_BCD[4]    = {17, 16, 39, 38}; // BCD 1/2/4/8, INPUT_PULLUP, active-low
static constexpr uint8_t PIN_NRF_CE    = 14;
static constexpr uint8_t PIN_NRF_CSN   = 0;
static constexpr uint8_t PIN_NRF_IRQ   = 9;            // optional (hier gepollt)

// ------------------------------ Konstanten -----------------------------------
static constexpr double  G        = 9.80665;
static constexpr double  GYRO_LSB = 65.5;             // LSB/(deg/s), FS_SEL=1
static constexpr double  ACC_LSB  = 8192.0;           // LSB/g,       AFS_SEL=1
static constexpr double  DEG2RAD  = 3.14159265358979323846 / 180.0;
static constexpr uint8_t MPU_ADDR = 0x68;
static constexpr uint8_t MPU_PWR_MGMT_1 = 0x6B, MPU_GYRO_CONFIG = 0x1B,
                         MPU_ACCEL_CONFIG = 0x1C, MPU_ACCEL_XOUT_H = 0x3B;
static constexpr uint32_t LINK_TIMEOUT_MS = 100;      // Failsafe
static constexpr uint32_t BIAS_MS = 3000;             // Gyro-Bias-Mittelung
static const uint64_t     NRF_BCAST_ADDR = 0xE7E7E7E7E7ULL;
static constexpr int      ESC_MIN = 512, ESC_MAX = 1024;

// ------------------------------ Globals --------------------------------------
static MCU               g_mcu;
static MCU::ExtU_mcu_T   g_U;                // wird jeden Tick befuellt
static pkt::PktCmd       g_cmd;              // letztes gueltiges Kommando (ZOH)
static double            g_gyro_bias[3] = {0,0,0};
static uint8_t           g_own_id = 0;
static volatile bool     g_tick = false;
static volatile uint32_t g_t_last_rx = 0;    // millis() des letzten gueltigen Pakets
static RF24              g_radio(PIN_NRF_CE, PIN_NRF_CSN);
static IntervalTimer     g_timer;

// ------------------------------ MPU-6050 -------------------------------------
static void mpu_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
// Burst-Read 0x3B..: liefert Gyro & Acc bereits in {B} (R_bs) und SI-Einheiten.
// gyro[rad/s], acc[m/s^2]. KEINE Bias-Subtraktion hier (macht der Caller).
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
    // state = Batterie-FSM-state (mcu_DW.state). Sammel-LED an, solange nicht 0.
    digitalWrite(PIN_LED, state != 0);
    // TODO: state -> {25/50/75/100 %}-Muster dekodieren, sobald Pins feststehen.
    digitalWrite(PIN_STAT_100, state /* == VOLL ? */ ? HIGH : LOW);
}

// ------------------------------ nRF ------------------------------------------
// Broadcast pollen: nur Pakete mit passender ID annehmen (Design A).
static void nrf_poll() {
    uint8_t buf[pkt::N_BYTES];
    while (g_radio.available()) {
        g_radio.read(buf, pkt::N_BYTES);
        if (!pkt::id_matches(buf, g_own_id)) continue; // Fremdpaket verwerfen
        pkt::unpack(buf, g_cmd);                       // ZOH: g_cmd haelt bis zum naechsten
        g_t_last_rx = millis();
    }
}

// ------------------------------ Startup-FSM ----------------------------------
static void esc_calibrate_and_arm() {
    // TODO: Timings/Sequenz an deine ESCs anpassen (Endpunkte = Flug-Endpunkte!).
    for (int i=0;i<4;++i) analogWrite(PIN_PWM[i], ESC_MAX); // max fuer Einlernen
    delay(3000);                                            // Piep abwarten
    for (int i=0;i<4;++i) analogWrite(PIN_PWM[i], ESC_MIN); // min -> Endpunkt gelernt
    delay(3000);                                            // Arming
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

// ------------------------------ setup ----------------------------------------
void setup() {
    for (int i=0;i<4;++i) pinMode(PIN_PWM[i], OUTPUT);
    pinMode(PIN_LED, OUTPUT); pinMode(PIN_STAT_100, OUTPUT);
    for (int i=0;i<4;++i) pinMode(PIN_BCD[i], INPUT_PULLUP);

    // ESC/OneShot125-PWM: count 512..1024 == 125..250 us
    analogWriteResolution(12);
    for (int i=0;i<4;++i) analogWriteFrequency(PIN_PWM[i], 1000);
    for (int i=0;i<4;++i) analogWrite(PIN_PWM[i], ESC_MIN);   // sofort sicher = min

    analogReadResolution(12);                                // batt_count roh 0..4095

    Wire.begin(); Wire.setClock(400000);                     // Fast-Mode Pflicht (1 kHz-Budget)
    mpu_write(MPU_PWR_MGMT_1, 0x00);                         // wake
    mpu_write(MPU_GYRO_CONFIG, 0x08);                        // FS_SEL=1 (+-500 dps)
    mpu_write(MPU_ACCEL_CONFIG, 0x08);                       // AFS_SEL=1 (+-4 g)

    g_own_id = read_bcd_id();

    // nRF Broadcast, Auto-Ack AUS (Design A) auf SPI1 (26/1/27 = Default-SPI1-Pins)
    g_radio.begin(&SPI1);
    g_radio.setAutoAck(false);
    g_radio.setPayloadSize(pkt::N_BYTES);
    g_radio.setDataRate(RF24_1MBPS);
    // g_radio.setChannel(76);
    g_radio.openReadingPipe(1, NRF_BCAST_ADDR);
    g_radio.startListening();

    // Startup-Sequenz (Drohne am Boden, still): ESC einlernen+armen, Gyro-Bias
    esc_calibrate_and_arm();
    estimate_gyro_bias();

    // Init-Kommando = sicher (kein Schub), bis das erste Paket kommt
    g_cmd = pkt::PktCmd{};                                   // alles 0
    g_cmd.q_des[0]=1; g_cmd.q_ref[0]=1; g_cmd.q_ext[0]=1;   // Identitaets-Quats
    g_cmd.estop = 2;                                         // bis Link steht: gekillt
    g_t_last_rx = 0;

    MCU::initialize();
    g_timer.begin(on_tick, 1000);                            // 1 kHz, us
}

// ------------------------------ loop -----------------------------------------
void loop() {
    nrf_poll();                                              // Pakete jederzeit annehmen
    if (!g_tick) return;
    g_tick = false;

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

    // 3) batt_count: rohe 12-bit counts (Volt-Umrechnung macht S6 im Modell)
    g_U.batt_count = (double)analogRead(PIN_BATT_V);

    // 4) Ein step()
    g_mcu.setExternalInputs(&g_U);
    g_mcu.step();
    const MCU::ExtY_mcu_T& y = g_mcu.getExternalOutputs();

    // 5) Aktorik: throttle -> OneShot125, led-state -> LEDs
    esc_write_all(y.throttle);
    drive_leds(y.led);
}
