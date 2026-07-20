// safety_helpers.h — ABI der Onboard-Safety-Leaf-Funktionen.
//
// safety_overspeed.m und safety_battery.m halten persistenten Zustand. MATLAB
// Coder emittiert dafuer je ein <fn>_initialize() (nullt die statics) plus den
// Step. Dieses Muster bilden wir hier als reset()+step() ab, damit derselbe
// Testkoerper gegen Referenz und generierten Code laeuft:
//   overspeed_reset()  ~  safety_overspeed_initialize()
//   overspeed_step()   ~  safety_overspeed()
// Beim Wechsel auf Codegen ersetzt ein duenner Shim reset/step durch die
// generierten Symbole (die Params kommen in die von Coder erzeugte struct).
#ifndef SAFETY_HELPERS_H
#define SAFETY_HELPERS_H
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Overspeed (safety_overspeed.m) ------------------------------------
struct OverspeedParams {
    double   omega_max;         // [rad/s]
    uint16_t debounce_N;        // >=1
    bool     use_norm;          // true: ||gyro|| ; false: per-Achse
    double   tilt_cos_min;      // = cos(tilt_max): Trigger, wenn cos(Kippwinkel) darunter
    uint16_t tilt_debounce_N;   // >=1
};
void overspeed_reset(void);
void overspeed_step(const double gyro_corr[3], const double q_hat[4],
                    uint8_t estop, bool ack, bool btn,
                    const OverspeedParams* p,
                    bool* kill, uint8_t* fault_src, double dbg[3]);

// ---- Battery (safety_battery.m) ----------------------------------------
struct BatteryParams {
    double batt_k, batt_b, batt_alpha;
    double V_warn, V_crit, V_floor, V_hyst;
};
void battery_reset(void);
void battery_step(double batt_count, const BatteryParams* p,
                  uint8_t* led, bool* batt_land, double* V_filt);

#ifdef __cplusplus
}
#endif
#endif  // SAFETY_HELPERS_H
