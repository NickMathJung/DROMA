// mcu_flat_io.hpp — Adapter Golden-CSV <-> generierte MCU_FLAT-ABI (mcu_flat.h).
// Pendant zu mcu_io.hpp (Kaskade). Die einzige Stelle, die gegen den Coder-
// Output abgeglichen werden muss: die Feldnamen unten
// (ExtU: Bus_IMU_k / Bus_Cmd_flat_l / batt_count / btn_ack ;
//  ExtY: rotor_cmd[4] / led / throttle[4]).
// Eigener Namespace sitl_flat, damit test_mcu_model (Kaskade) und
// test_mcu_flat_model nie um Symbole konkurrieren.
#ifndef SITL_MCU_FLAT_IO_HPP
#define SITL_MCU_FLAT_IO_HPP
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include "mcu_flat.h"

namespace sitl_flat {

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
inline void wire_inputs(MCU_FLAT::ExtU_mcu_flat_T& u, const NamedCsv& g, std::size_t r) {
    for (int i = 0; i < 3; ++i)
        u.Bus_IMU_k.imu_gyro[i] = g.get(r, "Bus_IMU.imu_gyro." + std::to_string(i+1));
    for (int i = 0; i < 3; ++i)
        u.Bus_IMU_k.imu_acc[i]  = g.get(r, "Bus_IMU.imu_acc."  + std::to_string(i+1));
    for (int i = 0; i < 3; ++i)
        u.Bus_Cmd_flat_l.mocap_pos[i] = g.get(r, "Bus_Cmd_flat.mocap_pos." + std::to_string(i+1));
    for (int i = 0; i < 4; ++i)
        u.Bus_Cmd_flat_l.q_ext[i]     = g.get(r, "Bus_Cmd_flat.q_ext."     + std::to_string(i+1));
    for (int i = 0; i < 3; ++i)
        u.Bus_Cmd_flat_l.p_ref[i]     = g.get(r, "Bus_Cmd_flat.p_ref."     + std::to_string(i+1));
    for (int i = 0; i < 3; ++i)
        u.Bus_Cmd_flat_l.v_ref[i]     = g.get(r, "Bus_Cmd_flat.v_ref."     + std::to_string(i+1));
    for (int i = 0; i < 3; ++i)
        u.Bus_Cmd_flat_l.a_ref[i]     = g.get(r, "Bus_Cmd_flat.a_ref."     + std::to_string(i+1));
    for (int i = 0; i < 3; ++i)
        u.Bus_Cmd_flat_l.j_ref[i]     = g.get(r, "Bus_Cmd_flat.j_ref."     + std::to_string(i+1));
    for (int i = 0; i < 3; ++i)
        u.Bus_Cmd_flat_l.s_ref[i]     = g.get(r, "Bus_Cmd_flat.s_ref."     + std::to_string(i+1));
    for (int i = 0; i < 3; ++i)
        u.Bus_Cmd_flat_l.yaw_ref[i]   = g.get(r, "Bus_Cmd_flat.yaw_ref."   + std::to_string(i+1));
    u.Bus_Cmd_flat_l.estop = static_cast<uint8_T>(g.get(r, "Bus_Cmd_flat.estop.1"));
    u.Bus_Cmd_flat_l.ack   = (g.get(r, "Bus_Cmd_flat.ack.1") != 0.0);
    u.batt_count           = g.get(r, "batt_count.1");
    u.btn_ack              = g.has("btn_ack.1") ? (g.get(r, "btn_ack.1") != 0.0) : false;
}

// --- ExtY rotor_cmd vs Golden -> groesste Abweichung ------------------------
inline double diff_rotor(const MCU_FLAT::ExtY_mcu_flat_T& y, const NamedCsv& g, std::size_t r) {
    double w = 0.0;
    for (int i = 0; i < 4; ++i) {
        double d = std::abs(y.rotor_cmd[i] - g.get(r, "rotor_cmd." + std::to_string(i+1)));
        if (d > w) w = d;
    }
    return w;
}

// --- ExtY led (uint8, exakt) vs Golden --------------------------------------
inline double diff_led(const MCU_FLAT::ExtY_mcu_flat_T& y, const NamedCsv& g, std::size_t r) {
    return std::abs(static_cast<double>(y.led) - g.get(r, "led.1"));
}

// --- ExtY throttle[4] ([0,100]) vs Golden -> groesste Abweichung ------------
inline double diff_throttle(const MCU_FLAT::ExtY_mcu_flat_T& y, const NamedCsv& g, std::size_t r) {
    double w = 0.0;
    for (int i = 0; i < 4; ++i) {
        double d = std::abs(y.throttle[i] - g.get(r, "throttle." + std::to_string(i+1)));
        if (d > w) w = d;
    }
    return w;
}

}  // namespace sitl_flat
#endif
