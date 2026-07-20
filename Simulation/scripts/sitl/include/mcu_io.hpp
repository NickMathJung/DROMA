// mcu_io.hpp — Adapter Golden-CSV <-> generierte MCU-ABI (mcu_types.h).
// Die einzige Stelle, die gegen den Coder-Output abgeglichen werden muss: die
// Feldnamen unten (ExtU: Bus_IMU_k / Bus_Cmd_l / batt_count ; ExtY: rotor_cmd[4]).
#ifndef SITL_MCU_IO_HPP
#define SITL_MCU_IO_HPP
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include "mcu.h"

namespace sitl {

struct NamedCsv {
    std::vector<std::string> header;            // Spaltennamen (inkl. k,t)
    std::unordered_map<std::string,std::size_t> idx;
    std::vector<std::vector<double>> rows;       // je Zeile alle Felder als double
    double get(std::size_t r, const std::string& name) const {
        auto it = idx.find(name);
        if (it == idx.end())
            throw std::runtime_error("Golden-Spalte fehlt: " + name);
        return rows[r][it->second];
    }
    bool has(const std::string& name) const { return idx.count(name) > 0; }
};

inline NamedCsv read_named_csv(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Golden-CSV nicht gefunden: " + path);
    NamedCsv c;
    std::string line;
    if (!std::getline(f, line)) throw std::runtime_error("Leere CSV: " + path);
    { std::stringstream ss(line); std::string cell;
      while (std::getline(ss, cell, ',')) {
          c.idx[cell] = c.header.size(); c.header.push_back(cell); } }
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line); std::string cell;
        std::vector<double> v; v.reserve(c.header.size());
        while (std::getline(ss, cell, ',')) v.push_back(std::stod(cell));
        if (v.size() != c.header.size())
            throw std::runtime_error("Spaltenzahl != Header in " + path);
        c.rows.push_back(std::move(v));
    }
    return c;
}

// --- Golden-Zeile -> ExtU (column-major .1/.2/.. wie im Logger) -------------
inline void wire_inputs(MCU::ExtU_mcu_T& u, const NamedCsv& g, std::size_t r) {
    for (int i = 0; i < 3; ++i)
        u.Bus_IMU_k.imu_gyro[i] = g.get(r, "Bus_IMU.imu_gyro." + std::to_string(i+1));
    for (int i = 0; i < 3; ++i)
        u.Bus_IMU_k.imu_acc[i]  = g.get(r, "Bus_IMU.imu_acc."  + std::to_string(i+1));
    u.Bus_Cmd_l.F_des = g.get(r, "Bus_Cmd.F_des.1");
    for (int i = 0; i < 4; ++i)
        u.Bus_Cmd_l.q_des[i]    = g.get(r, "Bus_Cmd.q_des."    + std::to_string(i+1));
    for (int i = 0; i < 4; ++i)
        u.Bus_Cmd_l.q_ref[i]    = g.get(r, "Bus_Cmd.q_ref."    + std::to_string(i+1));
    for (int i = 0; i < 3; ++i)
        u.Bus_Cmd_l.Omega_ref[i]= g.get(r, "Bus_Cmd.Omega_ref."+ std::to_string(i+1));
    for (int i = 0; i < 3; ++i)
        u.Bus_Cmd_l.tau_ref[i]  = g.get(r, "Bus_Cmd.tau_ref."  + std::to_string(i+1));
    for (int i = 0; i < 4; ++i)
        u.Bus_Cmd_l.q_ext[i]    = g.get(r, "Bus_Cmd.q_ext."    + std::to_string(i+1));
    u.Bus_Cmd_l.estop = static_cast<uint8_T>(g.get(r, "Bus_Cmd.estop.1"));
    u.Bus_Cmd_l.ack   = (g.get(r, "Bus_Cmd.ack.1") != 0.0);
    u.batt_count      = g.get(r, "batt_count.1");
    // btn_ack (Teensy-Taster, active-low) -> ge-OR-t mit Bus_Cmd.ack im Modell.
    // has()-Guard: aeltere Golden ohne die Spalte fallen auf false zurueck.
    u.btn_ack         = g.has("btn_ack.1") ? (g.get(r, "btn_ack.1") != 0.0) : false;
}

// --- ExtY rotor_cmd vs Golden -> groesste Abweichung ------------------------
inline double diff_rotor(const MCU::ExtY_mcu_T& y, const NamedCsv& g, std::size_t r) {
    double w = 0.0;
    for (int i = 0; i < 4; ++i) {
        double d = std::abs(y.rotor_cmd[i] - g.get(r, "rotor_cmd." + std::to_string(i+1)));
        if (d > w) w = d;
    }
    return w;
}

// --- ExtY led (uint8, exakt) vs Golden --------------------------------------
inline double diff_led(const MCU::ExtY_mcu_T& y, const NamedCsv& g, std::size_t r) {
    return std::abs(static_cast<double>(y.led) - g.get(r, "led.1"));
}

// --- ExtY throttle[4] ([0,100]) vs Golden -> groesste Abweichung ------------
inline double diff_throttle(const MCU::ExtY_mcu_T& y, const NamedCsv& g, std::size_t r) {
    double w = 0.0;
    for (int i = 0; i < 4; ++i) {
        double d = std::abs(y.throttle[i] - g.get(r, "throttle." + std::to_string(i+1)));
        if (d > w) w = d;
    }
    return w;
}

}  // namespace sitl
#endif
