// quat_helpers_ref.cpp — bit-treue Referenz-Portierung der MATLAB-Quelle.
//
// Zweck: (1) Der C++-Golden-Test laeuft ohne MATLAB gruen (CI, host-side loop).
//        (2) Diese Datei ist das Vorbild, gegen das der spaeter generierte Code
//            diffen muss; weicht Golden von Codegen ab, ist es ein Codegen-Bug.
//
// Indexierung column-major:  Rm(i,j) == R(i+1,j+1) in MATLAB-Notation.
#include "quat_helpers.h"
#include <cmath>

#define Rm(i, j) R[(i) + 3 * (j)]  // column-major access, 0-basiert

void dcm2quat_local(const double R[9], double q[4]) {
    // Shepperd-Methode, 1:1 aus dcm2quat_local.m (row/col via Rm gemappt).
    const double tr = Rm(0, 0) + Rm(1, 1) + Rm(2, 2);
    double q0, q1, q2, q3, S;
    if (tr > 0.0) {
        S  = 2.0 * std::sqrt(tr + 1.0);
        q0 = 0.25 * S;
        q1 = (Rm(1, 2) - Rm(2, 1)) / S;
        q2 = (Rm(2, 0) - Rm(0, 2)) / S;
        q3 = (Rm(0, 1) - Rm(1, 0)) / S;
    } else if ((Rm(0, 0) > Rm(1, 1)) && (Rm(0, 0) > Rm(2, 2))) {
        S  = 2.0 * std::sqrt(1.0 + Rm(0, 0) - Rm(1, 1) - Rm(2, 2));
        q0 = (Rm(1, 2) - Rm(2, 1)) / S;
        q1 = 0.25 * S;
        q2 = (Rm(0, 1) + Rm(1, 0)) / S;
        q3 = (Rm(0, 2) + Rm(2, 0)) / S;
    } else if (Rm(1, 1) > Rm(2, 2)) {
        S  = 2.0 * std::sqrt(1.0 + Rm(1, 1) - Rm(0, 0) - Rm(2, 2));
        q0 = (Rm(2, 0) - Rm(0, 2)) / S;
        q1 = (Rm(0, 1) + Rm(1, 0)) / S;
        q2 = 0.25 * S;
        q3 = (Rm(1, 2) + Rm(2, 1)) / S;
    } else {
        S  = 2.0 * std::sqrt(1.0 + Rm(2, 2) - Rm(0, 0) - Rm(1, 1));
        q0 = (Rm(0, 1) - Rm(1, 0)) / S;
        q1 = (Rm(0, 2) + Rm(2, 0)) / S;
        q2 = (Rm(1, 2) + Rm(2, 1)) / S;
        q3 = 0.25 * S;
    }
    const double n = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q[0] = q0 / n;
    q[1] = q1 / n;
    q[2] = q2 / n;
    q[3] = q3 / n;
}

void quat2dcm_local(const double q[4], double R[9]) {
    const double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    // Aerospace-Formel (row-major-Mathematik), abgelegt column-major via Rm.
    Rm(0, 0) = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    Rm(0, 1) = 2.0 * (q1 * q2 + q0 * q3);
    Rm(0, 2) = 2.0 * (q1 * q3 - q0 * q2);
    Rm(1, 0) = 2.0 * (q1 * q2 - q0 * q3);
    Rm(1, 1) = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    Rm(1, 2) = 2.0 * (q2 * q3 + q0 * q1);
    Rm(2, 0) = 2.0 * (q1 * q3 + q0 * q2);
    Rm(2, 1) = 2.0 * (q2 * q3 - q0 * q1);
    Rm(2, 2) = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}

void quatMul(const double a[4], const double c[4], double r[4]) {
    r[0] = a[0] * c[0] - a[1] * c[1] - a[2] * c[2] - a[3] * c[3];
    r[1] = a[0] * c[1] + a[1] * c[0] + a[2] * c[3] - a[3] * c[2];
    r[2] = a[0] * c[2] - a[1] * c[3] + a[2] * c[0] + a[3] * c[1];
    r[3] = a[0] * c[3] + a[1] * c[2] - a[2] * c[1] + a[3] * c[0];
}

void quatConj(const double a[4], double r[4]) {
    r[0] =  a[0];
    r[1] = -a[1];
    r[2] = -a[2];
    r[3] = -a[3];
}

void quatRotate(const double q[4], const double vn[3], double vb[3]) {
    double R[9];
    quat2dcm_local(q, R);  // vb = R(q)*vn ; R column-major
    for (int i = 0; i < 3; ++i)
        vb[i] = Rm(i, 0) * vn[0] + Rm(i, 1) * vn[1] + Rm(i, 2) * vn[2];
}

#undef Rm
