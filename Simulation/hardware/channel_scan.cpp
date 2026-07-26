// channel_scan.cpp — nRF24 Kanal-/Interferenz-Scanner (Bench).
//
// Zweck: den ruhigsten der 126 nRF-Kanaele empirisch finden. Hintergrund ist die
// S-3-Diagnose: der Link verliert ~63 % der Frames (rx~37 / gaps~63 bei emit=100),
// obwohl Sender + Simulink sauber mit 100 Hz emittieren -> reiner On-Air-Verlust.
// Erstverdacht: Kanal 76 (=2476 MHz) liegt mitten im oberen WLAN-Bereich (WLAN-Kanal
// 13, 2461..2483 MHz). Dieser Scan misst, welche Kanaele belegt sind, damit wir den
// Link empirisch statt geraten umziehen.
//
// Verfahren (kanonischer Carrier-Detect-Scan): pro Kanal kurz auf Empfang gehen und
// testCarrier() abfragen. Traeger vorhanden -> Zaehler hoch. Ueber viele Durchlaeufe
// ergibt sich pro Kanal ein Belegungsmass 0..f (hex, gesaettigt). Ausserdem alle paar
// Sweeps eine Rangliste der ruhigsten Kanaele + explizit der Wert von Kanal 76.
//
// nRF-Verdrahtung IDENTISCH zu gcs_sender.cpp / drone_hal.cpp:
//   SPI1 (SCK27/MOSI26/MISO1), CE14, CSN0.  DataRate 1MBPS wie der Link, damit der
//   Scan die Bandbreite sieht, die der Link tatsaechlich belegt.
//
// Wichtig: NUR mit laufendem Stoerumfeld aussagekraeftig — also WLAN/BT an lassen wie
// im echten Betrieb. Der eigene Sender (gcs_sender) darf ruhig senden; dann taucht
// Kanal 76 als "belegt" auf und bestaetigt, dass der Scan funktioniert.

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#ifdef printf
#undef printf                 // RF24 (Teensy) macht '#define printf Serial.printf'
#endif

static constexpr uint8_t PIN_NRF_CE = 14, PIN_NRF_CSN = 0;
static constexpr int     NUM_CH   = 126;      // 2400..2525 MHz (Kanal = MHz - 2400)
static constexpr int     PASSES   = 200;      // Carrier-Abfragen pro Kanal je Sweep
static constexpr int     LINK_CH  = 76;       // aktueller Link-Kanal (Referenz)

static RF24    g_radio(PIN_NRF_CE, PIN_NRF_CSN);
static uint8_t g_val[NUM_CH];

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {}

    SPI1.setMOSI(26); SPI1.setMISO(1); SPI1.setSCK(27);
    SPI1.begin();
    bool ok = g_radio.begin(&SPI1);
    g_radio.setAutoAck(false);
    g_radio.setDataRate(RF24_1MBPS);          // == Link
    g_radio.startListening();
    g_radio.stopListening();

    Serial.printf("[scan] nRF ok=%d chip=%d — Carrier-Scan 0..125 (2400..2525 MHz).\n",
                  (int)ok, (int)g_radio.isChipConnected());
    Serial.println("[scan] Belegung je Kanal 0..f (hex, hoeher=mehr Traeger). "
                   "WLAN-Kanaele ~ nRF 2/22 (ch1), 27 (ch6), 52 (ch11), 76 (ch13).");
}

// Ein Sweep: jeden Kanal PASSES-mal auf Traeger pruefen, Treffer zaehlen (max 0xF).
static void sweep_once() {
    memset(g_val, 0, sizeof(g_val));
    for (int rep = 0; rep < PASSES; ++rep) {
        for (int ch = 0; ch < NUM_CH; ++ch) {
            g_radio.setChannel(ch);
            g_radio.startListening();
            delayMicroseconds(130);            // Einschwingen (nRF: ~130 us RX-Settle)
            g_radio.stopListening();
            if (g_radio.testCarrier() && g_val[ch] < 0xF) ++g_val[ch];
        }
    }
}

void loop() {
    sweep_once();

    // Zeile 1: klassisches Histogramm (ein Hex-Zeichen je Kanal).
    Serial.print("  ");
    for (int ch = 0; ch < NUM_CH; ++ch) Serial.printf("%x", g_val[ch]);
    Serial.println();

    // Zeile 2: die 5 ruhigsten Kanaele + Referenzwert Kanal 76.
    int order[NUM_CH]; for (int i = 0; i < NUM_CH; ++i) order[i] = i;
    for (int i = 0; i < NUM_CH - 1; ++i)               // simple selection sort nach g_val
        for (int j = i + 1; j < NUM_CH; ++j)
            if (g_val[order[j]] < g_val[order[i]]) { int t=order[i]; order[i]=order[j]; order[j]=t; }
    Serial.print("  ruhigste: ");
    for (int i = 0; i < 5; ++i)
        Serial.printf("ch%d(%dMHz)=%x  ", order[i], 2400 + order[i], g_val[order[i]]);
    Serial.printf("| Link ch%d=%x\n", LINK_CH, g_val[LINK_CH]);
    Serial.println("  ---");
}
