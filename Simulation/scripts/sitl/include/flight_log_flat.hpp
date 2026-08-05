// flight_log_flat.hpp — Blackbox-Format der Flatness-Firmware (single source of truth).
//
// Zweck: waehrend des Flugs NICHTS auf die SD schreiben. Der 1-kHz-Tick haengt in
// loop(), und eine SD-Karte legt fuer ihre interne Verwaltung gelegentlich 50-100 ms
// Pausen ein -- das waere ein Regelungsproblem, kein Datenproblem. Stattdessen:
//   Tick  -> zwei memcpy in einen Ringpuffer im RAM2 (DMAMEM), < 1 us
//   danach-> auf Kommando einmal komplett auf die SD, mit stehenden Motoren.
//
// Zwei getrennte Ringe statt eines gemischten Stroms, damit kein Record-Tag noetig
// ist und die Zeitachse implizit bleibt:
//   FAST @1 kHz : IMU (das ist die Rate, die fuer Vibrationsspektren zaehlt)
//   SLOW @100 Hz: alles, was sich langsam aendert (Mocap, Referenz, Drossel, Akku)
// Ein Fast-Record wird exakt einmal pro Tick geschrieben. Sein Index IM RING ist
// damit der Tick-Zaehler modulo Ringlaenge; der Header traegt den absoluten Zaehler
// des JUENGSTEN Records, woraus der Leser jeden Zeitstempel exakt rekonstruiert.
//
// int16 statt float bei der IMU ist kein Genauigkeitsverlust: die Skalen unten sind
// so gewaehlt, dass ein LSB des Logs feiner ist als ein LSB des Sensors
// (Gyro FS_SEL=1: 1/65.5 dps = 2.67e-4 rad/s; Acc AFS_SEL=1: 1/8192 g = 1.2e-3 m/s^2).
#pragma once
#include <stdint.h>
#include <string.h>

namespace flog {

static const char     MAGIC[8] = {'D','R','O','M','A','F','L'};   // + '\0' = 8
static const uint16_t VERSION  = 2;   // v2: Reglerzustaende im Slow-Record

// Festkomma-Skalen [LSB je SI-Einheit]. Stehen im Header -> Leser ist selbstbeschreibend.
// Exakt die Kehrwerte der Sensorskalen aus drone_hal_flat (GYRO_LSB=65.5 LSB/dps,
// ACC_LSB=8192 LSB/g, G=9.80665). Damit ist ein Log-LSB genau ein Sensor-LSB: die
// int16-Ablage kostet keine Aufloesung, und der int16-Bereich deckt genau den
// Messbereich (+-500 dps bzw. +-4 g) ab. Eine frei gewaehlte Skala kann das nicht
// besser -- die MPU nutzt ueber ihren FSR bereits den vollen int16-Bereich.
static const float GYRO_SCALE = 3752.8736f;   // = 65.5 * 180/pi  -> +-8.731 rad/s
static const float ACC_SCALE  =  835.3517f;   // = 8192 / 9.80665 -> +-39.23 m/s^2

// Reglerzustaende (v2). Bereiche aus flatness_ctrl/flatness_khat, jeweils mit
// Reserve auf den naechsten glatten Wert; die Aufloesung ist ueberall mindestens
// vier Groessenordnungen feiner als das, was man im Verlauf ablesen will.
static const float KHAT_SCALE = 20000.0f;  // k_hat in [0.5, 1.5] geklemmt -> +-1.64
static const float F_SCALE    =   800.0f;  // Schub bis ~40 N            -> +-40.96 N
static const float AINT_SCALE =    80.0f;  // AINT_MAX = 400             -> +-409.6
static const float UFB_SCALE  =     8.0f;  // UFB_MAX = 700, roh groesser-> +-4096

static inline int16_t q15(double v, float scale) {
    double s = v * (double)scale;
    if (s >  32767.0) return  32767;
    if (s < -32768.0) return -32768;
    return (int16_t)(s >= 0 ? s + 0.5 : s - 0.5);
}
static inline double unq15(int16_t v, float scale) { return (double)v / (double)scale; }

#pragma pack(push, 1)

// 12 B @1 kHz. Gyro ist BIASFREI und im Body-Frame (also genau das, was das Modell
// sieht) -- fuer die Notch-Auslegung ist das die richtige Groesse, denn dort sitzt
// die Vibration, die der Regler verstaerkt.
struct RecFast {
    int16_t gyro[3];   // rad/s  * GYRO_SCALE
    int16_t acc[3];    // m/s^2  * ACC_SCALE
};

// 74 B @100 Hz.
struct RecSlow {
    uint32_t tick;         // absoluter 1-kHz-Tickzaehler (Quervergleich zum Fast-Ring)
    float    mocap_pos[3];
    float    p_ref[3];
    float    q_ext[4];
    uint16_t thr[4];       // Drossel in 0.01 %  (0..10000)
    uint16_t batt_count;   // roher ADC-Wert
    uint8_t  estop;
    uint8_t  led;
    uint8_t  ack;
    uint8_t  flags;        // Bit0: Link-Timeout aktiv, Bit1: Ringe uebergelaufen
    // --- v2: aus dem mcu_flat-Ausgang dbg = [k_hat; F; aint(3); u_fb_raw(3)] ----
    // Genau die Groessen, die sonst in den persistenten Zustaenden von
    // flatness_ctrl/flatness_khat verschwinden. u_fb_raw ist die Rueckfuehrung VOR
    // der UFB_MAX-Klemme: |u_fb_raw| > 700 heisst, der Regler steht in der Saettigung.
    int16_t  k_hat;        // * KHAT_SCALE
    int16_t  F;            // * F_SCALE   [N]
    int16_t  aint[3];      // * AINT_SCALE
    int16_t  ufb[3];       // * UFB_SCALE [m/s^4]
};

// 64 B. Steht am Anfang der Dumpdatei; danach n_fast RecFast, dann n_slow RecSlow,
// beide bereits in chronologische Reihenfolge gebracht (Ring aufgeloest).
struct Header {
    char     magic[8];
    uint16_t version;
    uint8_t  own_id;
    uint8_t  hal_mode;       // 0=BENCH 1=THRUST 2=FLIGHT
    uint32_t ts_fast_us;     // 1000
    uint16_t div_slow;       // 10 -> 100 Hz
    uint16_t rec_fast_len;   // sizeof(RecFast)
    uint16_t rec_slow_len;   // sizeof(RecSlow)
    uint16_t pad0;
    float    gyro_scale;
    float    acc_scale;
    uint32_t n_fast;         // Anzahl geschriebener Fast-Records im Dump
    uint32_t n_slow;
    uint32_t tick_first;     // Tickzaehler des ERSTEN Fast-Records im Dump
    uint32_t tick_last;      // Tickzaehler des LETZTEN
    uint32_t t_dump_ms;      // millis() beim Dump
    uint8_t  reserved[12];
};

#pragma pack(pop)

static const uint16_t REC_FAST_LEN = 12;
static const uint16_t REC_SLOW_LEN = 74;
static const uint16_t HEADER_LEN   = 64;

// Groessenannahmen hart absichern: das Format wandert 1:1 auf die Karte und wird
// vom MATLAB-/Python-Leser mit genau diesen Offsets gelesen.
static_assert(sizeof(RecFast) == REC_FAST_LEN, "RecFast != 12 B");
static_assert(sizeof(RecSlow) == REC_SLOW_LEN, "RecSlow != 74 B");
static_assert(sizeof(Header)  == HEADER_LEN,   "Header  != 64 B");

inline void fill_header(Header& h, uint8_t own_id, uint8_t hal_mode,
                        uint32_t n_fast, uint32_t n_slow,
                        uint32_t tick_first, uint32_t tick_last, uint32_t t_dump_ms) {
    memset(&h, 0, sizeof(h));
    memcpy(h.magic, MAGIC, 8);
    h.version      = VERSION;
    h.own_id       = own_id;
    h.hal_mode     = hal_mode;
    h.ts_fast_us   = 1000;
    h.div_slow     = 10;
    h.rec_fast_len = REC_FAST_LEN;
    h.rec_slow_len = REC_SLOW_LEN;
    h.gyro_scale   = GYRO_SCALE;
    h.acc_scale    = ACC_SCALE;
    h.n_fast       = n_fast;
    h.n_slow       = n_slow;
    h.tick_first   = tick_first;
    h.tick_last    = tick_last;
    h.t_dump_ms    = t_dump_ms;
}

inline bool header_valid(const Header& h) {
    return memcmp(h.magic, MAGIC, 8) == 0 && h.version == VERSION
        && h.rec_fast_len == REC_FAST_LEN && h.rec_slow_len == REC_SLOW_LEN;
}

// --- Ringpuffer -------------------------------------------------------------
// Bewusst ohne Allokation: der Speicher kommt vom Aufrufer (auf dem Teensy ein
// DMAMEM-Array in RAM2). push() ist das, was im Tick laeuft -- ein memcpy und
// zwei Increments, nichts weiter.
template <typename Rec>
struct Ring {
    Rec*     buf  = nullptr;
    uint32_t cap  = 0;     // Anzahl Records
    uint32_t head = 0;     // naechster Schreibindex
    uint32_t n    = 0;     // gefuellte Records (<= cap)
    uint32_t seq  = 0;     // absolut geschriebene Records (laeuft weiter nach Wrap)

    void init(Rec* mem, uint32_t capacity) {
        buf = mem; cap = capacity; head = 0; n = 0; seq = 0;
    }
    inline void push(const Rec& r) {
        if (!cap) return;
        buf[head] = r;
        head = (head + 1 == cap) ? 0 : head + 1;
        if (n < cap) ++n;
        ++seq;
    }
    bool wrapped() const { return seq > cap; }
    // Index des i-ten Records in chronologischer Reihenfolge (i = 0 .. n-1).
    uint32_t chrono(uint32_t i) const {
        uint32_t start = (n == cap) ? head : 0;
        uint32_t k = start + i;
        return (k >= cap) ? k - cap : k;
    }
    // Absoluter Zaehler des i-ten chronologischen Records.
    uint32_t seq_of(uint32_t i) const { return seq - n + i; }
};

} // namespace flog
