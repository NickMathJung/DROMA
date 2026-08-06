// Erzeugt eine deterministische Blackbox-Beispieldatei aus den ECHTEN Strukturen
// von flight_log_flat.hpp. Gegenstueck: read_flight_log_flat.m liest sie und
// prueft die Werte -- damit ist die Byte-Interpretation zwischen Firmware und
// MATLAB-Leser gekreuzt geprueft und nicht nur beidseitig behauptet.
//
// Aufruf: gen_flight_log_sample <ausgabedatei>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "flight_log_flat.hpp"

int main(int argc, char** argv) {
    const char* out = (argc > 1) ? argv[1] : "flight_log_sample.bin";

    // Ring absichtlich UEBERLAUFEN lassen: so wird auch die Chronologie nach dem
    // Wrap mitgeprueft, und genau dieser Fall tritt bei jedem langen Flug ein.
    const uint32_t CAP_F = 100, CAP_S = 10;
    std::vector<flog::RecFast> mf(CAP_F);
    std::vector<flog::RecSlow> ms(CAP_S);
    flog::Ring<flog::RecFast> rf; rf.init(mf.data(), CAP_F);
    flog::Ring<flog::RecSlow> rs; rs.init(ms.data(), CAP_S);

    const uint32_t N_TICK = 250;                 // 2.5 Umlaeufe im Fast-Ring
    for (uint32_t k = 1; k <= N_TICK; ++k) {
        flog::RecFast a{};
        // Reproduzierbare, aber nichttriviale Werte in SI -> ueber q15 quantisiert.
        a.gyro[0] = flog::q15( 0.001 * (double)k,       flog::GYRO_SCALE);
        a.gyro[1] = flog::q15(-0.002 * (double)k,       flog::GYRO_SCALE);
        a.gyro[2] = flog::q15( 0.0005 * (double)k,      flog::GYRO_SCALE);
        a.acc[0]  = flog::q15( 0.01 * (double)k,        flog::ACC_SCALE);
        a.acc[1]  = flog::q15(-0.01 * (double)k,        flog::ACC_SCALE);
        a.acc[2]  = flog::q15( 9.81,                    flog::ACC_SCALE);
        rf.push(a);

        if (k % 10 == 0) {
            flog::RecSlow b{};
            b.tick = k;
            for (int i = 0; i < 3; ++i) b.mocap_pos[i] = 0.1f * (float)(k + i);
            for (int i = 0; i < 3; ++i) b.p_ref[i]     = 0.2f * (float)(k + i);
            b.q_ext[0] = 1.0f; b.q_ext[1] = 0.01f * (float)k;
            b.q_ext[2] = 0.02f * (float)k; b.q_ext[3] = 0.03f * (float)k;
            for (int i = 0; i < 4; ++i) b.thr[i] = (uint16_t)(1000 + 100*i + k);
            b.batt_count = (uint16_t)(900 + k);
            b.estop = (uint8_t)(k % 3);
            b.led   = (uint8_t)(k % 2);
            b.ack   = (uint8_t)((k / 10) % 2);
            b.flags = 0x02;                      // "Ring uebergelaufen"
            b.k_hat = flog::q15(0.90 + 0.001 * (double)k,   flog::KHAT_SCALE);
            b.F     = flog::q15(9.66 + 0.01  * (double)k,   flog::F_SCALE);
            for (int i = 0; i < 3; ++i) b.aint[i] = flog::q15(-100.0 + 10.0*(double)i + (double)k, flog::AINT_SCALE);
            for (int i = 0; i < 3; ++i) b.ufb[i]  = flog::q15( 500.0 - 50.0*(double)i - (double)k, flog::UFB_SCALE);
            b.w_adapt = flog::q15(0.004 * (double)k, flog::W_SCALE);   // rampt 0.04 .. 1.0
            rs.push(b);
        }
    }

    flog::Header h;
    flog::fill_header(h, 2, 2, rf.n, rs.n, N_TICK - rf.n + 1, N_TICK, 424242);

    FILE* f = fopen(out, "wb");
    if (!f) { fprintf(stderr, "kann %s nicht schreiben\n", out); return 1; }
    fwrite(&h, 1, sizeof(h), f);
    // Chronologisch, exakt wie dump_ring_raw in der Firmware.
    for (uint32_t i = 0; i < rf.n; ++i) fwrite(&rf.buf[rf.chrono(i)], 1, sizeof(flog::RecFast), f);
    for (uint32_t i = 0; i < rs.n; ++i) fwrite(&rs.buf[rs.chrono(i)], 1, sizeof(flog::RecSlow), f);
    fclose(f);

    printf("%s: %u fast + %u slow, tick %u..%u\n", out, (unsigned)rf.n, (unsigned)rs.n,
           (unsigned)h.tick_first, (unsigned)h.tick_last);
    return 0;
}
