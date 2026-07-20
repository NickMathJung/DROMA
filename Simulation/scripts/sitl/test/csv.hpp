// csv.hpp — winziger CSV-Reader fuer die Golden-Vektoren.
// Kein Fremd-Dependency; parst nur, was verify_quat_codegen.py schreibt
// (Header-Zeile + Zahlen als %.17g, erste Spalte = id-String).
#ifndef SITL_CSV_HPP
#define SITL_CSV_HPP

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace sitl {

struct Row {
    std::string id;
    std::vector<double> v;  // alle numerischen Felder nach der id, in Spaltenreihenfolge
};

inline std::vector<Row> read_csv(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Golden-CSV nicht gefunden: " + path);
    std::vector<Row> rows;
    std::string line;
    std::getline(f, line);  // Header verwerfen
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string cell;
        Row r;
        bool first = true;
        while (std::getline(ss, cell, ',')) {
            if (first) { r.id = cell; first = false; }
            else       { r.v.push_back(std::stod(cell)); }
        }
        rows.push_back(std::move(r));
    }
    return rows;
}

// Golden speichert R row-major (R11,R12,R13,R21,...). Die Codegen-ABI erwartet
// R column-major. Diese Funktion baut aus 9 aufeinanderfolgenden CSV-Werten
// (ab Offset off) das column-major Array, das dcm2quat_local(...) erwartet.
inline void row9_to_colmajor(const std::vector<double>& v, std::size_t off,
                             double R_col[9]) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R_col[i + 3 * j] = v[off + 3 * i + j];  // (i,j) row-major -> col-major
}

// Vergleich eines column-major Ergebnisses R_col gegen die row-major Golden-Werte.
inline double max_abs_diff_R(const double R_col[9], const std::vector<double>& v,
                             std::size_t off) {
    double w = 0.0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double d = std::abs(R_col[i + 3 * j] - v[off + 3 * i + j]);
            if (d > w) w = d;
        }
    return w;
}

}  // namespace sitl
#endif  // SITL_CSV_HPP
