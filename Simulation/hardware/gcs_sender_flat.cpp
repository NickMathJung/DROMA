// gcs_sender_flat.cpp — Sende-Teensy der FLATNESS-Variante: USB-Frame -> 2x nRF.
//
// Pendant zu gcs_sender.cpp (Kaskade), bewusst als EIGENE Datei: die Kaskade
// bleibt unangetastet und jederzeit flashbar (--upload-sender), Flatness laeuft
// ueber --upload-sender-flat. Nur EINER der beiden Sketches darf auf dem
// Sende-Teensy liegen — sie sprechen unterschiedliche OTA-Formate.
//
// Rolle: ID-Durchreicher zwischen Simulink-GCS und den Drohnen.
//   USB-Serial: gcsf-Frame [sync|id|Bus_Cmd_flat(float32)|estop|ack|crc8] (gcs_frame_flat.hpp)
//   -> gcsf::parse -> pktf::pack(Bus_Cmd_flat, id, seq[id]) (mcu_flat_packet.hpp)
//   -> radio.write(A,27) + radio.write(B,27) auf gemeinsame Broadcast-Adresse.
// Haelt seq pro Drohne; BEIDE Frames eines Zyklus tragen dieselbe seq (der
// Empfaenger paart darueber, siehe pktf::Assembler).
//
// Die nRF-Params sind identisch zum Drohnen-HAL (drone_hal_flat.cpp):
//   Adresse 0xE7E7E7E7E7, Kanal 76, RF24_250KBPS, Auto-Ack aus, 27-B-Payload.
//   250 kbit/s bleibt (10 dB Empfindlichkeit, war der Fix gegen ~63% On-Air-
//   Verlust). Funklast: 2 Frames x 1.16 ms = 2.3 ms je Drohne und Zyklus,
//   bei 100 Hz und 2 Drohnen also ~46% Kanalauslastung — Reserve vorhanden,
//   ab 3 Drohnen wird es eng (dann j_ref/s_ref kuerzen oder Frame B halbieren).
//   SPI1 (SCK27/MOSI26/MISO1), CE14, CSN0.

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "gcs_frame_flat.hpp"   // gcsf::parse / GcsCmdFlat
#include "mcu_flat_packet.hpp"  // pktf::pack / CmdFlat (gemeinsame Quelle mit der Drohne)

static constexpr uint8_t  PIN_NRF_CE = 14, PIN_NRF_CSN = 0, PIN_NRF_IRQ = 9;
static constexpr uint8_t  NRF_CHANNEL = 76;
static const uint64_t     NRF_BCAST_ADDR = 0xE7E7E7E7E7ULL;

static RF24    g_radio(PIN_NRF_CE, PIN_NRF_CSN);
static uint8_t g_seq[16] = {0};        // seq je Drohne-id (BCD 0..15)

// GcsCmdFlat (float32) -> pktf::CmdFlat (double). Feld-fuer-Feld (== Bus_Cmd_flat).
static void widen(const gcsf::GcsCmdFlat& s, pktf::CmdFlat& d) {
    for (int i = 0; i < 3; ++i) d.mocap_pos[i] = s.mocap_pos[i];
    for (int i = 0; i < 4; ++i) d.q_ext[i]     = s.q_ext[i];
    for (int i = 0; i < 3; ++i) d.p_ref[i]     = s.p_ref[i];
    for (int i = 0; i < 3; ++i) d.v_ref[i]     = s.v_ref[i];
    for (int i = 0; i < 3; ++i) d.a_ref[i]     = s.a_ref[i];
    for (int i = 0; i < 3; ++i) d.j_ref[i]     = s.j_ref[i];
    for (int i = 0; i < 3; ++i) d.s_ref[i]     = s.s_ref[i];
    for (int i = 0; i < 3; ++i) d.yaw_ref[i]   = s.yaw_ref[i];
    d.estop = s.estop;
    d.ack   = (s.ack != 0);
}

// Ein vollstaendiger, CRC-gepruefter Frame -> quantisieren + beide Haelften funken.
static void forward_frame(const uint8_t frame[gcsf::SIZE]) {
    gcsf::GcsCmdFlat gc; uint8_t id;
    if (!gcsf::parse(frame, gc, id)) return;    // Sync/CRC schlecht -> verwerfen
    if (id > 15) return;                        // ausserhalb BCD-Bereich
    pktf::CmdFlat cmd; widen(gc, cmd);
    uint8_t bufA[pktf::SIZE], bufB[pktf::SIZE];
    pktf::pack(cmd, id, g_seq[id]++, bufA, bufB);   // gleiche seq fuer A und B
    // Reihenfolge A dann B: A traegt Mocap + Positionsreferenz (der Pfad, der bei
    // knappem Budget zuerst ankommen soll); B ist reine Vorsteuerung.
    g_radio.write(bufA, pktf::SIZE);            // Auto-Ack aus -> kehrt nach TX zurueck
    g_radio.write(bufB, pktf::SIZE);

    // Bring-up-Heartbeat: LED toggelt nur bei gueltigen (Sync+CRC-ok) Frames von
    // Simulink -> blinkt = USB+Parse ok (Problem ggf. RF); dunkel = USB/Format-Problem.
    static uint16_t n = 0; static bool led = false;
    if ((++n % 5) == 0) { led = !led; digitalWrite(LED_BUILTIN, led); }
}

// --- USB-Serial: byteweiser Sync-Hunt (resynct nach jeder Stoerung) ----------
// NUR die frischeste vollstaendige Frame PRO id funken — Begruendung wie in
// gcs_sender.cpp: ein Echtzeit-Setpoint-Link darf keine veraltete Warteschlange
// abspielen (Alt-Frames mit frischer seq sehen fuer den Empfaenger gueltig aus).
// Pro id statt global, damit im Schwarmbetrieb keine Drohne verhungert.
static void serial_pump() {
    static uint8_t buf[gcsf::SIZE];
    static int idx = 0;
    static uint8_t st = 0;                       // 0=HUNT0, 1=HUNT1, 2=FILL
    uint8_t  latest[16][gcsf::SIZE];             // frischeste Frame je id (BCD 0..15)
    bool     have[16] = {};
    while (Serial.available()) {
        uint8_t b = (uint8_t)Serial.read();
        switch (st) {
            case 0: if (b == gcsf::SYNC0) { buf[0] = b; st = 1; } break;
            case 1:
                if (b == gcsf::SYNC1)      { buf[1] = b; idx = 2; st = 2; }
                else if (b == gcsf::SYNC0) { buf[0] = b; }        // AA AA... -> im HUNT1 bleiben
                else                       { st = 0; }
                break;
            default:
                buf[idx++] = b;
                if (idx >= gcsf::SIZE) {                          // vollstaendige Frame:
                    uint8_t fid = buf[gcsf::off::ID];             // CRC prueft forward_frame
                    if (fid < 16) {
                        for (int i = 0; i < gcsf::SIZE; ++i) latest[fid][i] = buf[i];
                        have[fid] = true;                         // merken, noch NICHT senden
                    }
                    st = 0;
                }
                break;
        }
    }
    for (int i = 0; i < 16; ++i)
        if (have[i]) forward_frame(latest[i]);   // pro Drain je id nur die neueste
}

void setup() {
    Serial.begin(1000000);                       // USB-CDC: Rate egal, aber definiert
    pinMode(LED_BUILTIN, OUTPUT);                // Heartbeat (Pin 13, frei — SPI1 nutzt SCK27)

    // Teensy: SPI1-Pins explizit + SPI1.begin() VOR RF24, sonst haengt begin(&SPI1).
    SPI1.setMOSI(26); SPI1.setMISO(1); SPI1.setSCK(27);
    SPI1.begin();
    g_radio.begin(&SPI1);
    g_radio.setAutoAck(false);
    g_radio.setPayloadSize(pktf::SIZE);          // 27 B, BEIDE Frame-Typen gleich gross
    g_radio.setDataRate(RF24_250KBPS);           // MUSS mit drone_hal_flat.cpp uebereinstimmen!
    g_radio.setChannel(NRF_CHANNEL);
    g_radio.openWritingPipe(NRF_BCAST_ADDR);
    g_radio.stopListening();                     // TX-Modus
}

void loop() {
    serial_pump();
}
