// Generiert von run_mcu_recert.m aus quadcop.p_from_omega_sq — bitte nicht editieren.
// throttle = clamp(polyval(P_THROTTLE, rotor_cmd^2), 0, 100).
#ifndef THROTTLE_POLY_HPP
#define THROTTLE_POLY_HPP
namespace mcuref {
static constexpr int    P_THROTTLE_N   = 3;
static constexpr double P_THROTTLE[3] = { -2.9813898214245336e-13, 1.2315874872894866e-05, 8.4040477510595064 };
static constexpr double THROTTLE_MIN = 0.0, THROTTLE_MAX = 100.0;
}  // namespace mcuref
#endif
