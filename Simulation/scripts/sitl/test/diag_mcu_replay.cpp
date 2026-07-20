// diag_mcu_replay.cpp — Ursachenanalyse fuer Golden-Divergenz.
// Replay des generierten MCU-Codes gegen golden_mcu_io.csv; findet den ersten
// Tick mit |dq|>TOL und zeigt: (a) ob es ein Einzelspike oder dauerhaft ist,
// (b) got-vs-exp um den Tick herum, (c) welche Eingangsspalten sich an genau
// diesem Tick gegenueber dem Vortick aendern, also das koinzidente Ereignis.
#include <array>
#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include "mcu_io.hpp"
using namespace sitl;
 
#ifndef GOLDEN_MCU_CSV
#define GOLDEN_MCU_CSV "data/golden_mcu_io.csv"
#endif
#ifndef DIAG_TOL
#define DIAG_TOL 1e-9
#endif
 
int main() {
    printf("Golden-Pfad (einkompiliert): %s\n", GOLDEN_MCU_CSV);
    NamedCsv g;
    try { g = read_named_csv(GOLDEN_MCU_CSV); }
    catch (const std::exception& e) { printf("FEHLER: %s\n", e.what()); return 2; }
    const double TOL = DIAG_TOL;
    printf("gelesen: %zu Zeilen, %zu Spalten.\n", g.rows.size(), g.header.size());
 
    // Eingangsspalten (alles ausser k,t,rotor_cmd.*,led.*) fuer die Delta-Analyse.
    std::vector<std::string> incols;
    for (auto& h : g.header) {
        if (h=="k"||h=="t") continue;
        if (h.rfind("rotor_cmd.",0)==0) continue;
        if (h.rfind("led.",0)==0) continue;
        incols.push_back(h);
    }
 
    MCU obj; obj.initialize();
    std::vector<std::array<double,5>> got(g.rows.size());
    long first_bad=-1, last_bad=-1, nbad=0; double worst=0; long worst_row=-1;
    long first_led_bad=-1;
    for (std::size_t r=0; r<g.rows.size(); ++r) {
        MCU::ExtU_mcu_T u{}; wire_inputs(u,g,r);
        obj.setExternalInputs(&u); obj.step();
        const auto& y = obj.getExternalOutputs();
        got[r] = {y.rotor_cmd[0],y.rotor_cmd[1],y.rotor_cmd[2],y.rotor_cmd[3],(double)y.led};
        double d = diff_rotor(y,g,r);
        if (d>worst){worst=d;worst_row=(long)r;}
        if (d>TOL){ if(first_bad<0)first_bad=(long)r; last_bad=(long)r; ++nbad; }
        if (diff_led(y,g,r)!=0.0 && first_led_bad<0) first_led_bad=(long)r;
    }
 
    printf("=== Replay-Diagnose (%zu Ticks, TOL=%.0e) ===\n", g.rows.size(), TOL);
    printf("rotor_cmd: worst |dq|=%.6g @ Tick %ld (t=%.4f)\n",
           worst, worst_row, worst_row>=0?g.get(worst_row,"t"):0.0);
    if (first_bad<0){ printf("KEINE rotor-Divergenz > TOL. led erst bei Tick %ld.\n",
                             first_led_bad); return 0; }
    printf("erster Abweich-Tick: %ld (t=%.4f) | letzter: %ld | Anzahl>TOL: %ld / %zu\n",
           first_bad, g.get(first_bad,"t"), last_bad, nbad, g.rows.size());
    printf("Charakter: %s\n", (nbad==1) ? "EINZELSPIKE (erholt sich)"
           : (last_bad-first_bad+1==nbad) ? "DAUERHAFT ab erstem Tick"
           : "MEHRERE Bursts");
    if (first_led_bad>=0) printf("led divergiert ab Tick %ld.\n", first_led_bad);
 
    long b = first_bad;
    printf("\n--- rotor_cmd got vs exp um Tick %ld ---\n", b);
    for (long r=std::max(0L,b-2); r<=std::min((long)g.rows.size()-1,b+3); ++r) {
        printf("t=%.4f k=%ld  ", g.get(r,"t"), r);
        for (int i=0;i<4;++i)
            printf("r%d[%.6g/%.6g] ", i+1, got[r][i], g.get(r,"rotor_cmd."+std::to_string(i+1)));
        printf(" led[%d/%.0f]\n", (int)got[r][4], g.get(r,"led.1"));
    }
 
    printf("\n--- Eingangs-Spalten, die sich Tick %ld->%ld aendern ---\n", b-1, b);
    if (b>=1) {
        int changed=0;
        for (auto& c : incols) {
            double a=g.get(b-1,c), n=g.get(b,c);
            if (a!=n){ printf("  %-22s  %.9g -> %.9g\n", c.c_str(), a, n); ++changed; }
        }
        if (!changed) printf("  (keine — Ursache liegt in MCU-internem Zustand, nicht im Eingang)\n");
    }
    return 1;
}