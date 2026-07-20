// quat_helpers.h — ABI der Quaternion-Helfer.
//
// Diese Signaturen sind identisch zu dem, was MATLAB Coder aus dcm2quat_local.m /
// quat2dcm_local.m / quatMul.m / quatConj.m / quatRotate.m erzeugt.
//
// Speicherordnung, wichtig: R[9] ist column-major, so wie MATLAB Coder es
// emittiert:  R[i + 3*j] == R(i+1, j+1)  (0-basiert i,j). Die Testvektoren-CSV
// speichert R dagegen row-major (R11,R12,R13,R21,...). Diese Umrechnung passiert
// nur im CSV-Adapter (test/csv.hpp), nie hier.
//
// q = [q0 q1 q2 q3], quat2dcm_local(q) ist die DCM Inertial->Koerper;
// dcm2quat_local ist deren Inverse.
#ifndef QUAT_HELPERS_H
#define QUAT_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

// R (3x3, column-major) -> q (4x1), normiert.  Shepperd, 4 Zweige.
void dcm2quat_local(const double R[9], double q[4]);

// q (4x1) -> R (3x3, column-major).
void quat2dcm_local(const double q[4], double R[9]);

// Hamilton-Produkt r = a (x) c.
void quatMul(const double a[4], const double c[4], double r[4]);

// Konjugierte r = [a0; -a1; -a2; -a3].
void quatConj(const double a[4], double r[4]);

// vb = R(q) * vn.
void quatRotate(const double q[4], const double vn[3], double vb[3]);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // QUAT_HELPERS_H
