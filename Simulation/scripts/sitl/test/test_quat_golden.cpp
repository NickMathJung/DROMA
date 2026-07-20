// test_quat_golden.cpp — C++-Golden-Test der Quaternion-Helfer.
//
// Spiegelt verify_quat_codegen.m (run_test / run_quatops) nach GoogleTest:
//  - Helfer(Golden-Input) == Golden-Output  (Quaternionen bis auf Vorzeichen)
//  - Property-Round-Trips ueber Zufalls-Rotationen
// Laeuft gegen quat_helpers_ref.cpp (Standin) ODER den generierten Code
// (CMake -DQUAT_IMPL=codegen) — identischer Testkoerper.
#include "quat_helpers.h"
#include "csv.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include <string>

// Toleranz: Compile-Define GOLDEN_TOL (Default 1e-9 = MATLAB-vs-Golden aus
// verify_quat_codegen.m). Fuer den Codegen-Diff bewusst eng lassen — eine lose
// Toleranz versteckt genau die ULP-Divergenzen, die der Test fangen soll.
#ifndef GOLDEN_TOL
#define GOLDEN_TOL 1e-9
#endif
static constexpr double kTol = GOLDEN_TOL;

#ifndef GOLDEN_DIR
#define GOLDEN_DIR "."
#endif
static std::string gpath(const char* f) { return std::string(GOLDEN_DIR) + "/" + f; }

static double q_dist(const double a[4], const double b[4]) {  // bis auf Vorzeichen
    double sp = 0, sm = 0;
    for (int i = 0; i < 4; ++i) { double p=a[i]-b[i], m=a[i]+b[i]; sp+=p*p; sm+=m*m; }
    return std::min(std::sqrt(sp), std::sqrt(sm));
}

// ---------------------------------------------------------------- dcm2quat
TEST(QuatGolden, Dcm2Quat_MatchesGolden) {
    auto rows = sitl::read_csv(gpath("test_data_quat.csv"));
    ASSERT_FALSE(rows.empty());
    double worst = 0; std::string worst_id;
    int branch_hist[4] = {0,0,0,0};
    for (const auto& r : rows) {
        double Rc[9]; sitl::row9_to_colmajor(r.v, 0, Rc);   // v: R(9), q(4), branch
        double qref[4] = { r.v[9], r.v[10], r.v[11], r.v[12] };
        int branch = static_cast<int>(r.v[13]);
        branch_hist[branch]++;
        double q[4]; dcm2quat_local(Rc, q);
        double e = q_dist(q, qref);
        if (e > worst) { worst = e; worst_id = r.id; }
        EXPECT_LT(e, kTol) << "Fall " << r.id << " (Zweig " << branch << ")";
    }
    // Alle 4 Shepperd-Zweige muessen in den Golden vertreten sein.
    for (int b = 0; b < 4; ++b)
        EXPECT_GT(branch_hist[b], 0) << "Shepperd-Zweig " << b << " nicht abgedeckt";
    RecordProperty("worst_id", worst_id);
    std::printf("  dcm2quat worst |dq| = %.3e (Fall %s), Zweige [%d %d %d %d]\n",
                worst, worst_id.c_str(),
                branch_hist[0], branch_hist[1], branch_hist[2], branch_hist[3]);
}

// ---------------------------------------------------------------- quat2dcm
// Nur aussagekraeftig, wenn quat2dcm_local aus dem echten mcu-Code stammt (noch
// offen). Der Standin nutzt dieselbe Aerospace-Formel, der Test ist also scharf.
TEST(QuatGolden, Quat2Dcm_MatchesGolden) {
    auto rows = sitl::read_csv(gpath("test_data_quat.csv"));
    double worst = 0;
    for (const auto& r : rows) {
        double qref[4] = { r.v[9], r.v[10], r.v[11], r.v[12] };
        double Rc[9]; quat2dcm_local(qref, Rc);
        worst = std::max(worst, sitl::max_abs_diff_R(Rc, r.v, 0));
    }
    EXPECT_LT(worst, kTol);
    std::printf("  quat2dcm worst |dR| = %.3e\n", worst);
}

// ---------------------------------------------------------------- quatMul
TEST(QuatGolden, QuatMul_MatchesGolden) {
    auto rows = sitl::read_csv(gpath("test_data_quatmul.csv"));
    ASSERT_FALSE(rows.empty());
    double worst = 0;
    for (const auto& r : rows) {           // v: a(4), c(4), r(4)
        double a[4]={r.v[0],r.v[1],r.v[2],r.v[3]};
        double c[4]={r.v[4],r.v[5],r.v[6],r.v[7]};
        double ref[4]={r.v[8],r.v[9],r.v[10],r.v[11]};
        double out[4]; quatMul(a,c,out);
        worst = std::max(worst, q_dist(out, ref));  // bis auf Vorzeichen
    }
    EXPECT_LT(worst, kTol);
    std::printf("  quatMul  worst |dq| = %.3e\n", worst);
}

// ---------------------------------------------------------------- quatConj
TEST(QuatGolden, QuatConj_MatchesGolden) {
    auto rows = sitl::read_csv(gpath("test_data_quatconj.csv"));
    ASSERT_FALSE(rows.empty());
    double worst = 0;
    for (const auto& r : rows) {           // v: a(4), r(4)  (Vorzeichen fix)
        double a[4]={r.v[0],r.v[1],r.v[2],r.v[3]};
        double ref[4]={r.v[4],r.v[5],r.v[6],r.v[7]};
        double out[4]; quatConj(a,out);
        for (int i=0;i<4;++i) worst = std::max(worst, std::abs(out[i]-ref[i]));
    }
    EXPECT_LT(worst, kTol);
    std::printf("  quatConj worst |d|  = %.3e\n", worst);
}

// ---------------------------------------------------------------- quatRotate
TEST(QuatGolden, QuatRotate_MatchesGolden) {
    auto rows = sitl::read_csv(gpath("test_data_quatrotate.csv"));
    ASSERT_FALSE(rows.empty());
    double worst = 0;
    for (const auto& r : rows) {           // v: q(4), vn(3), vb(3)
        double q[4]={r.v[0],r.v[1],r.v[2],r.v[3]};
        double vn[3]={r.v[4],r.v[5],r.v[6]};
        double ref[3]={r.v[7],r.v[8],r.v[9]};
        double vb[3]; quatRotate(q,vn,vb);
        for (int i=0;i<3;++i) worst = std::max(worst, std::abs(vb[i]-ref[i]));
    }
    EXPECT_LT(worst, kTol);
    std::printf("  quatRotate worst |dv|=%.3e\n", worst);
}

// -------------------------------------------------- Property-Round-Trips
// Entspricht run_roundtrips / run_quatops-Eigenschaften in verify_quat_codegen.m.
TEST(QuatProps, RoundTripsAndIdentities) {
    std::mt19937_64 rng(7);
    std::normal_distribution<double> N(0,1);
    auto randq = [&](double q[4]){
        double n=0; for(int i=0;i<4;++i){q[i]=N(rng); n+=q[i]*q[i];}
        n=std::sqrt(n); for(int i=0;i<4;++i) q[i]/=n;
    };
    const double I[4]={1,0,0,0};
    double wRTq=0,wRTR=0,wNorm=0,wId=0,wInv=0,wAssoc=0,wRot=0,wLen=0,wDcm=0;
    for (int it=0; it<20000; ++it) {
        double q[4]; randq(q);
        double R[9]; quat2dcm_local(q,R);
        double qo[4]; dcm2quat_local(R,qo);
        double nrm=0; for(int i=0;i<4;++i) nrm+=qo[i]*qo[i];
        wNorm=std::max(wNorm,std::abs(std::sqrt(nrm)-1.0));
        wRTq =std::max(wRTq, q_dist(qo,q));
        double R2[9]; quat2dcm_local(qo,R2);
        for(int i=0;i<9;++i) wRTR=std::max(wRTR,std::abs(R2[i]-R[i]));
        // Algebra
        double a[4]; randq(a); double c[4]; randq(c); double d[4]; randq(d);
        double t1[4]; quatMul(a,I,t1);              for(int i=0;i<4;++i) wId=std::max(wId,std::abs(t1[i]-a[i]));
        double ca[4]; quatConj(a,ca); double ii[4]; quatMul(a,ca,ii);
        wInv=std::max(wInv,q_dist(ii,I));
        double ac[4]; quatMul(a,c,ac); double acd[4]; quatMul(ac,d,acd);
        double cd[4]; quatMul(c,d,cd); double acd2[4]; quatMul(a,cd,acd2);
        for(int i=0;i<4;++i) wAssoc=std::max(wAssoc,std::abs(acd[i]-acd2[i]));
        // Rotation
        double v[3]={N(rng),N(rng),N(rng)};
        double vb[3]; quatRotate(a,v,vb);
        double vbb[3]; quatRotate(ca,vb,vbb);
        for(int i=0;i<3;++i) wRot=std::max(wRot,std::abs(vbb[i]-v[i]));
        double nv=std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
        double nvb=std::sqrt(vb[0]*vb[0]+vb[1]*vb[1]+vb[2]*vb[2]);
        wLen=std::max(wLen,std::abs(nvb-nv));
        double Ra[9]; quat2dcm_local(a,Ra);
        for(int i=0;i<3;++i){
            double dv=Ra[i]*v[0]+Ra[i+3]*v[1]+Ra[i+6]*v[2];
            wDcm=std::max(wDcm,std::abs(vb[i]-dv));
        }
    }
    EXPECT_LT(wRTq, 1e-8);   EXPECT_LT(wRTR, kTol);   EXPECT_LT(wNorm, 1e-12);
    EXPECT_LT(wId, 1e-12);   EXPECT_LT(wInv, 1e-12);  EXPECT_LT(wAssoc, 1e-12);
    EXPECT_LT(wRot, 1e-11);  EXPECT_LT(wLen, 1e-12);  EXPECT_LT(wDcm, 1e-12);
    std::printf("  props: RTq=%.1e RTR=%.1e |q|=%.1e id=%.1e inv=%.1e assoc=%.1e "
                "rot=%.1e len=%.1e dcm=%.1e\n",
                wRTq,wRTR,wNorm,wId,wInv,wAssoc,wRot,wLen,wDcm);
}
