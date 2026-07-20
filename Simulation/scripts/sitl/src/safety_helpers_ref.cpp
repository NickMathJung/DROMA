// safety_helpers_ref.cpp — bit-treue Referenz-Ports der Safety-Leaf-Funktionen.
// Latch-, Entprell- und EMA-Semantik exakt wie in den .m-Quellen.
#include "safety_helpers.h"
#include <cmath>

// --- safety_overspeed ---
namespace {
bool     os_latched  = false;
uint16_t os_cnt      = 0;
uint16_t os_tcnt     = 0;
bool     os_ack_prev = false;
bool     os_btn_prev = false;
uint8_t  os_src      = 0;
bool     os_inited   = false;
}

void overspeed_reset(void) {
    os_latched = false; os_cnt = 0; os_tcnt = 0;
    os_ack_prev = false; os_btn_prev = false; os_src = 0;
    os_inited = false;
}

void overspeed_step(const double gyro_corr[3], const double q_hat[4],
                    uint8_t estop, bool ack, bool btn,
                    const OverspeedParams* p,
                    bool* kill, uint8_t* fault_src, double dbg[3]) {
    if (!os_inited) {  // entspricht 'isempty(latched)' beim ersten Aufruf
        os_latched = false; os_cnt = 0; os_tcnt = 0;
        os_ack_prev = false; os_btn_prev = false; os_src = 0;
        os_inited = true;
    }
    const double g0 = gyro_corr[0], g1 = gyro_corr[1], g2 = gyro_corr[2];

    bool over_inst;
    if (p->use_norm) {
        over_inst = std::sqrt(g0*g0 + g1*g1 + g2*g2) > p->omega_max;
    } else {
        over_inst = (std::abs(g0) > p->omega_max) ||
                    (std::abs(g1) > p->omega_max) ||
                    (std::abs(g2) > p->omega_max);
    }

    const uint16_t Nreq = p->debounce_N;
    if (over_inst) { if (os_cnt < Nreq) os_cnt = static_cast<uint16_t>(os_cnt + 1); }
    else           { os_cnt = 0; }
    const bool over_deb = os_cnt >= Nreq;

    // Tilt: cos(Kippwinkel) = (w^2 - x^2 - y^2 + z^2) / |q|^2, normiert.
    const double qw = q_hat[0], qx = q_hat[1], qy = q_hat[2], qz = q_hat[3];
    const double n2 = qw*qw + qx*qx + qy*qy + qz*qz;
    bool tilt_inst;
    if (n2 < 1e-12) {
        tilt_inst = false;
    } else {
        const double cos_tilt = (qw*qw - qx*qx - qy*qy + qz*qz) / n2;
        tilt_inst = cos_tilt < p->tilt_cos_min;
    }
    const uint16_t Ntilt = p->tilt_debounce_N;
    if (tilt_inst) { if (os_tcnt < Ntilt) os_tcnt = static_cast<uint16_t>(os_tcnt + 1); }
    else           { os_tcnt = 0; }
    const bool tilt_deb = os_tcnt >= Ntilt;

    const bool hard_kill = (estop == 2);
    const bool btn_edge  = btn && !os_btn_prev;

    if (over_deb  && !os_latched) { os_latched = true; os_src = 1; }
    if (hard_kill && !os_latched) { os_latched = true; os_src = 2; }
    if (tilt_deb  && !os_latched) { os_latched = true; os_src = 3; }
    if (btn_edge  && !os_latched) { os_latched = true; os_src = 4; }

    const bool ack_edge = ack && !os_ack_prev;
    if (os_latched && ack_edge && !over_inst && !tilt_inst && (estop != 2) && !btn) {
        os_latched = false; os_cnt = 0; os_tcnt = 0; os_src = 0;
    }
    os_ack_prev = ack;
    os_btn_prev = btn;

    *kill      = os_latched;
    *fault_src = os_src;
    dbg[0] = static_cast<double>(os_cnt);
    dbg[1] = over_inst ? 1.0 : 0.0;
    dbg[2] = ack_edge  ? 1.0 : 0.0;
}

// --- safety_battery ---
namespace {
double  bt_Vf     = 0.0;
uint8_t bt_state  = 0;
bool    bt_landed = false;
bool    bt_inited = false;
}

void battery_reset(void) { bt_inited = false; }

void battery_step(double batt_count, const BatteryParams* p,
                  uint8_t* led, bool* batt_land, double* V_filt) {
    if (!bt_inited) {
        bt_Vf     = p->batt_k * batt_count + p->batt_b;  // Seed aus 1. Sample
        bt_state  = 0;
        bt_landed = false;
        bt_inited = true;
    }
    const double V_raw = p->batt_k * batt_count + p->batt_b;
    bt_Vf = bt_Vf + p->batt_alpha * (V_raw - bt_Vf);
    const double V = bt_Vf;

    switch (bt_state) {
        case 0:  // NORMAL
            if (V <= p->V_warn) bt_state = 1;
            break;
        case 1:  // WARN
            if      (V <= p->V_crit)                 bt_state = 2;
            else if (V >= p->V_warn + p->V_hyst)     bt_state = 0;
            break;
        default: // CRIT (2)
            if (V >= p->V_crit + p->V_hyst)          bt_state = 1;
            break;
    }
    if (V <= p->V_floor) bt_landed = true;  // sticky, kein Re-Arm

    *led       = bt_state;
    *batt_land = bt_landed;
    *V_filt    = V;
}
