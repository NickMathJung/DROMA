// gcs_sender.cpp — Sende-Teensy (Design A): USB-Frame -> nRF-Broadcast.
//
// Rolle: reiner ID-Durchreicher zwischen Simulink-GCS und den 3 Drohnen.
//   USB-Serial: gcs-Frame [sync|id|Bus_Cmd(float32)|estop|ack|crc8] (gcs_frame.hpp)
//   -> gcs::parse -> pkt::pack(Bus_Cmd, id, seq[id]) (mcu_packet.hpp)
//   -> radio.write(buf,29) auf gemeinsame Broadcast-Adresse.
// Haelt seq pro Drohne. Die Quantisierung (float32 -> int16/sm3) passiert erst hier.
//
// Die nRF-Params sind identisch zum Drohnen-HAL (drone_hal.cpp):
//   Adresse 0xE7E7E7E7E7, Kanal 76, RF24_1MBPS, Auto-Ack aus, 29-B-Payload.
//   SPI1 (SCK27/MOSI26/MISO1), CE14, CSN0 (Wiring des Sende-Teensy-Boards, an
//   dessen Schaltplan bestaetigen).

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "gcs_frame.hpp"    // gcs::parse / GcsCmd
#include "mcu_packet.hpp"   // pkt::pack / Cmd  (gemeinsame Quelle mit der Drohne)

static constexpr uint8_t  PIN_NRF_CE = 14, PIN_NRF_CSN = 0, PIN_NRF_IRQ = 9;
static constexpr uint8_t  NRF_CHANNEL = 76;
static const uint64_t     NRF_BCAST_ADDR = 0xE7E7E7E7E7ULL;

static RF24    g_radio(PIN_NRF_CE, PIN_NRF_CSN);
static uint8_t g_seq[16] = {0};        // seq je Drohne-id (BCD 0..15)

// GcsCmd (float32) -> pkt::Cmd (double). Feld-fuer-Feld (Reihenfolge == Bus_Cmd).
static void widen(const gcs::GcsCmd& s, pkt::Cmd& d) {
    d.F_des = s.F_des;
    for (int i = 0; i < 4; ++i) d.q_des[i] = s.q_des[i];
    for (int i = 0; i < 4; ++i) d.q_ref[i] = s.q_ref[i];
    for (int i = 0; i < 3; ++i) d.Omega_ref[i] = s.Omega_ref[i];
    for (int i = 0; i < 3; ++i) d.tau_ref[i] = s.tau_ref[i];
    for (int i = 0; i < 4; ++i) d.q_ext[i] = s.q_ext[i];
    d.estop = s.estop;
    d.ack   = (s.ack != 0);
}

// Ein vollstaendiger, CRC-gepruefter Frame -> quantisieren + broadcasten.
static void forward_frame(const uint8_t frame[gcs::SIZE]) {
    gcs::GcsCmd gc; uint8_t id;
    if (!gcs::parse(frame, gc, id)) return;     // Sync/CRC schlecht -> verwerfen
    if (id > 15) return;                        // ausserhalb BCD-Bereich
    pkt::Cmd cmd; widen(gc, cmd);
    uint8_t buf[pkt::SIZE];
    pkt::pack(cmd, id, g_seq[id]++, buf);       // seq pro Drohne, dann inkrementieren
    g_radio.write(buf, pkt::SIZE);              // Auto-Ack aus -> kehrt nach TX zurueck

    // Bring-up-Heartbeat: LED toggelt nur bei gueltigen (Sync+CRC-ok) Frames von
    // Simulink -> blinkt = USB+Parse ok (Problem ggf. RF); dunkel = USB/Format-Problem.
    static uint16_t n = 0; static bool led = false;
    if ((++n % 5) == 0) { led = !led; digitalWrite(LED_BUILTIN, led); }
}

// --- USB-Serial: byteweiser Sync-Hunt (resynct nach jeder Stoerung) ----------
static void serial_pump() {
    static uint8_t buf[gcs::SIZE];
    static int idx = 0;
    static uint8_t st = 0;                       // 0=HUNT0, 1=HUNT1, 2=FILL
    while (Serial.available()) {
        uint8_t b = (uint8_t)Serial.read();
        switch (st) {
            case 0: if (b == gcs::SYNC0) { buf[0] = b; st = 1; } break;
            case 1:
                if (b == gcs::SYNC1)      { buf[1] = b; idx = 2; st = 2; }
                else if (b == gcs::SYNC0) { buf[0] = b; }        // AA AA... -> im HUNT1 bleiben
                else                      { st = 0; }
                break;
            default:
                buf[idx++] = b;
                if (idx >= gcs::SIZE) { forward_frame(buf); st = 0; }
                break;
        }
    }
}

void setup() {
    Serial.begin(1000000);                       // USB-CDC: Rate egal, aber definiert
    pinMode(LED_BUILTIN, OUTPUT);                 // Heartbeat (Pin 13, frei — SPI1 nutzt SCK27)

    // Teensy: SPI1-Pins explizit + SPI1.begin() VOR RF24, sonst haengt begin(&SPI1).
    SPI1.setMOSI(26); SPI1.setMISO(1); SPI1.setSCK(27);
    SPI1.begin();
    g_radio.begin(&SPI1);
    g_radio.setAutoAck(false);
    g_radio.setPayloadSize(pkt::SIZE);
    g_radio.setDataRate(RF24_1MBPS);
    g_radio.setChannel(NRF_CHANNEL);
    g_radio.openWritingPipe(NRF_BCAST_ADDR);
    g_radio.stopListening();                     // TX-Modus
}

void loop() {
    serial_pump();
}
