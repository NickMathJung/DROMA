// test_safety.cpp — Safety-Golden-Test.
// Portiert die Invarianten aus verify_overspeed.m (S1-S9) und verify_battery.m
// (B1-B6) nach GoogleTest. Persistente Funktionen -> vor jedem Szenario reset().
// Laeuft gegen safety_helpers_ref.cpp bzw. (Shim) gegen den generierten Code.
#include "safety_helpers.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <random>
#include <vector>

// ------------------------------------------------------------ Overspeed
namespace {
// Tilt hier deaktivieren (cos_min=-2 -> Trigger nie), die Overspeed-Szenarien
// sollen nur die Drehraten-Logik pruefen.
OverspeedParams OSP{10.0, 4, false, -2.0, 1};

const double Q_LEVEL[4] = {1.0, 0.0, 0.0, 0.0};   // Nullrotation, kein Tilt

struct OsOut { std::vector<int> k, src; };
// Treibt die Sequenz (g: Nx3, estop: N, ack: N) -> kill/src pro Sample.
// Lage level, Taster nicht gedrueckt (die dedizierten Faelle siehe weiter unten).
OsOut drive(const std::vector<std::array<double,3>>& g,
            const std::vector<uint8_t>& estop,
            const std::vector<uint8_t>& ack,
            const OverspeedParams& p) {
    OsOut o; std::size_t n = g.size();
    for (std::size_t i=0;i<n;++i){
        bool kill; uint8_t src; double dbg[3];
        overspeed_step(g[i].data(), Q_LEVEL, estop[i], ack[i]!=0, false, &p, &kill,&src,dbg);
        o.k.push_back(kill?1:0); o.src.push_back(src);
    }
    return o;
}
// Vollstaendiger Treiber inkl. Lage q (Nx4) und Taster btn (N).
OsOut drive_full(const std::vector<std::array<double,3>>& g,
                 const std::vector<std::array<double,4>>& q,
                 const std::vector<uint8_t>& estop,
                 const std::vector<uint8_t>& ack,
                 const std::vector<uint8_t>& btn,
                 const OverspeedParams& p) {
    OsOut o; std::size_t n = g.size();
    for (std::size_t i=0;i<n;++i){
        bool kill; uint8_t src; double dbg[3];
        overspeed_step(g[i].data(), q[i].data(), estop[i], ack[i]!=0, btn[i]!=0,
                       &p, &kill,&src,dbg);
        o.k.push_back(kill?1:0); o.src.push_back(src);
    }
    return o;
}
std::vector<std::array<double,3>> rep(std::array<double,3> v,int n){
    return std::vector<std::array<double,3>>(n,v);
}
std::vector<std::array<double,4>> repq(std::array<double,4> v,int n){
    return std::vector<std::array<double,4>>(n,v);
}
std::vector<std::array<double,3>> cat(std::vector<std::array<double,3>> a,
                                      const std::vector<std::array<double,3>>& b){
    a.insert(a.end(),b.begin(),b.end()); return a;
}
// Quaternion fuer einen Kippwinkel um die Roll-Achse (x): q=[cos(a/2),sin(a/2),0,0].
std::array<double,4> tilt_q(double deg){
    double a = deg * 3.14159265358979323846 / 180.0;
    return {std::cos(a/2), std::sin(a/2), 0.0, 0.0};
}
} // namespace

TEST(Overspeed, S1_NoOverspeed) {
    overspeed_reset();
    auto o = drive(rep({1,1,1},20), std::vector<uint8_t>(20,0), std::vector<uint8_t>(20,0), OSP);
    for (int k : o.k) EXPECT_EQ(k,0);
}
TEST(Overspeed, S2_Nminus1_NoLatch) {
    overspeed_reset();
    auto g = cat(rep({20,0,0},3), rep({0,0,0},5));
    auto o = drive(g, std::vector<uint8_t>(8,0), std::vector<uint8_t>(8,0), OSP);
    for (int k : o.k) EXPECT_EQ(k,0);
}
TEST(Overspeed, S3_TripAtNth_Holds) {
    overspeed_reset();
    auto g = cat(rep({20,0,0},4), rep({0,0,0},10));
    auto o = drive(g, std::vector<uint8_t>(14,0), std::vector<uint8_t>(14,0), OSP);
    EXPECT_EQ(o.k[2],0);                       // 3. Sample noch nicht
    EXPECT_EQ(o.k[3],1);                        // Trip exakt am 4. (N-ten)
    for (std::size_t i=3;i<o.k.size();++i) EXPECT_EQ(o.k[i],1);
    EXPECT_EQ(o.src[3],1);                      // fault_src = overspeed
}
TEST(Overspeed, S4_AckDuringOverspeed_NoRearm) {
    overspeed_reset();
    auto g = cat(rep({20,0,0},4), rep({20,0,0},6));
    std::vector<uint8_t> a(10,0); for(int i=4;i<10;++i) a[i]=1;
    auto o = drive(g, std::vector<uint8_t>(10,0), a, OSP);
    for (std::size_t i=3;i<o.k.size();++i) EXPECT_EQ(o.k[i],1);
}
TEST(Overspeed, S5_AckEdge_Rearms) {
    overspeed_reset();
    auto g = cat(rep({20,0,0},4), rep({0,0,0},4));
    std::vector<uint8_t> a(8,0); a[6]=1; a[7]=1;
    auto o = drive(g, std::vector<uint8_t>(8,0), a, OSP);
    EXPECT_EQ(o.k[5],1);   // vor Flanke latched
    EXPECT_EQ(o.k[6],0);   // Flanke -> re-armed
    EXPECT_EQ(o.k[7],0);   // bleibt armed bei gehaltenem ack
}
TEST(Overspeed, S6_HeldAck_NoAutoRearm) {
    overspeed_reset();
    auto g = cat(cat(rep({0,0,0},2), rep({20,0,0},4)), rep({0,0,0},4));
    auto o = drive(g, std::vector<uint8_t>(10,0), std::vector<uint8_t>(10,1), OSP);
    EXPECT_EQ(o.k[5],1);
    for (std::size_t i=5;i<o.k.size();++i) EXPECT_EQ(o.k[i],1);  // keine Flanke
}
TEST(Overspeed, S7_HardKill_Immediate) {
    overspeed_reset();
    std::vector<uint8_t> e{2,2,2,2,0,0}, a{0,0,0,0,0,1};
    auto o = drive(rep({0,0,0},6), e, a, OSP);
    EXPECT_EQ(o.k[0],1); EXPECT_EQ(o.src[0],2);
    EXPECT_EQ(o.k[3],1);
    EXPECT_EQ(o.k[4],1);   // estop->0 allein re-armt nicht
    EXPECT_EQ(o.k[5],0);   // estop=0 + ack-Flanke -> re-armed
}
TEST(Overspeed, S8_KillDominatesLand) {
    overspeed_reset();
    auto g = cat(rep({20,0,0},4), rep({0,0,0},3));
    auto o = drive(g, std::vector<uint8_t>(7,1), std::vector<uint8_t>(7,0), OSP);
    EXPECT_EQ(o.k[3],1); EXPECT_EQ(o.src[3],1);  // Overspeed dominiert soft-land
}
// S9 braucht laufzeit-schaltbares use_norm. Bei codegen mit coder.Constant ist der
// Modus einkompiliert (per-Achse) -> S9 wird dort ausgeblendet. Fuer volle Abdeckung
// die Safety-Leafs mit Laufzeit-Params generieren (siehe README/gen_lib_codegen.m).
#ifndef SAFETY_CODEGEN_CONST_PARAMS
TEST(Overspeed, S9_NormVsPerAxis) {
    OverspeedParams sN{10.0,4,true,-2.0,1};   // Tilt aus
    overspeed_reset();
    auto o1 = drive(rep({7.5,7.5,0},4), std::vector<uint8_t>(4,0), std::vector<uint8_t>(4,0), sN); // ||.||=10.6
    EXPECT_EQ(o1.k[3],1);
    overspeed_reset();
    auto o2 = drive(rep({6.0,6.0,0},6), std::vector<uint8_t>(6,0), std::vector<uint8_t>(6,0), sN); // ||.||=8.49
    for (int k : o2.k) EXPECT_EQ(k,0);
}
#endif  // SAFETY_CODEGEN_CONST_PARAMS

// (S10 war der Arming-Idle-Interlock-Test; das Feature ist verworfen,
//  Begruendung im Schlusskommentar von safety_overspeed.m.)

// ------------------------------------------------------------ Tilt-Cutoff
namespace {
// Exakt die Schwellen, die gen_lib_codegen.m einkompiliert (cos(80 deg),
// Entprellung 80). So laufen diese Faelle sowohl gegen die Referenz als auch
// gegen den generierten Code; Trip erst am 80. gekippten Sample (Index 79).
const double TILT_COS80 = std::cos(80.0*3.14159265358979323846/180.0);
OverspeedParams TP{10.0, 4, false, TILT_COS80, 80};
} // namespace

TEST(Tilt, T1_TripAtNth_Holds) {
    overspeed_reset();
    auto q = repq(tilt_q(85.0), 82);             // 85 deg > 80 deg, ueber die Entprellung
    auto o = drive_full(rep({0,0,0},82), q, std::vector<uint8_t>(82,0),
                        std::vector<uint8_t>(82,0), std::vector<uint8_t>(82,0), TP);
    EXPECT_EQ(o.k[78],0);                         // vor dem 80. Sample
    EXPECT_EQ(o.k[79],1);                         // Trip am N-ten (80.)
    EXPECT_EQ(o.src[79],3);                       // fault_src = tilt
    for (std::size_t i=79;i<o.k.size();++i) EXPECT_EQ(o.k[i],1);
}
TEST(Tilt, T2_BelowThreshold_NoTrip) {
    overspeed_reset();
    auto q = repq(tilt_q(70.0), 90);             // 70 deg < 80 deg
    auto o = drive_full(rep({0,0,0},90), q, std::vector<uint8_t>(90,0),
                        std::vector<uint8_t>(90,0), std::vector<uint8_t>(90,0), TP);
    for (int k : o.k) EXPECT_EQ(k,0);
}
TEST(Tilt, T3_ShortTilt_NoTrip) {
    overspeed_reset();
    std::vector<std::array<double,4>> q = repq(tilt_q(85.0),79);  // 79 < Entprellung
    auto lv = repq({1.0,0.0,0.0,0.0},5); q.insert(q.end(),lv.begin(),lv.end());
    std::size_t n = q.size();
    auto o = drive_full(rep({0,0,0},(int)n), q, std::vector<uint8_t>(n,0),
                        std::vector<uint8_t>(n,0), std::vector<uint8_t>(n,0), TP);
    for (int k : o.k) EXPECT_EQ(k,0);
}
TEST(Tilt, T4_NoRearmWhileTilted) {
    overspeed_reset();
    std::vector<std::array<double,4>> q = repq(tilt_q(85.0),82);  // 0..81 gekippt
    auto lv = repq({1.0,0.0,0.0,0.0},3); q.insert(q.end(),lv.begin(),lv.end()); // 82..84 level
    std::size_t n = q.size();                     // 85
    std::vector<uint8_t> ack(n,0); ack[81]=1; ack[84]=1;  // Flanke @81 (gekippt), @84 (level)
    auto o = drive_full(rep({0,0,0},(int)n), q, std::vector<uint8_t>(n,0), ack,
                        std::vector<uint8_t>(n,0), TP);
    EXPECT_EQ(o.k[81],1);   // ack-Flanke bei noch gekippter Lage re-armt nicht
    EXPECT_EQ(o.k[84],0);   // level + ack-Flanke -> re-armed
}

// ------------------------------------------------------------ Taster (btn)
TEST(Button, BT1_EdgeKills) {
    overspeed_reset();
    std::vector<uint8_t> btn{0,1,1,1,1};
    auto o = drive_full(rep({0,0,0},5), repq({1.0,0,0,0},5),
                        std::vector<uint8_t>(5,0), std::vector<uint8_t>(5,0), btn, OSP);
    EXPECT_EQ(o.k[0],0);
    EXPECT_EQ(o.k[1],1);   // steigende Taster-Flanke killt
    EXPECT_EQ(o.src[1],4); // fault_src = taster
    for (std::size_t i=1;i<o.k.size();++i) EXPECT_EQ(o.k[i],1);
}
TEST(Button, BT2_HeldBlocksRearm) {
    overspeed_reset();
    std::vector<uint8_t> btn{0,1,1,1,0,0};
    std::vector<uint8_t> ack{0,0,1,0,0,1};
    auto o = drive_full(rep({0,0,0},6), repq({1.0,0,0,0},6),
                        std::vector<uint8_t>(6,0), ack, btn, OSP);
    EXPECT_EQ(o.k[2],1);   // ack-Flanke bei gehaltenem Taster -> kein Re-Arm
    EXPECT_EQ(o.k[5],0);   // Taster losgelassen + ack-Flanke -> re-armed
}
TEST(Button, BT3_ButtonDoesNotRearmOverspeed) {
    overspeed_reset();
    auto g = cat(rep({20,0,0},4), rep({0,0,0},4));   // Overspeed trippt
    std::vector<uint8_t> btn{0,0,0,0,0,1,0,0};       // Taster-Flanke @5
    std::vector<uint8_t> ack {0,0,0,0,0,0,0,1};      // Bus_Cmd.ack-Flanke @7
    auto o = drive_full(g, repq({1.0,0,0,0},8), std::vector<uint8_t>(8,0), ack, btn, OSP);
    EXPECT_EQ(o.k[3],1); EXPECT_EQ(o.src[3],1);      // Overspeed-Latch
    EXPECT_EQ(o.k[6],1);                             // Taster quittiert NICHT
    EXPECT_EQ(o.k[7],0);                             // erst Bus_Cmd.ack re-armt
}

// ------------------------------------------------------------ Battery
namespace {
BatteryParams make_batt() {
    const double Ts=1.0/100.0, tau=0.7;
    BatteryParams p;
    p.batt_k = 3.3*18.182/4095.0;  p.batt_b = 0.0;
    p.batt_alpha = 1.0 - std::exp(-Ts/tau);
    p.V_warn=14.0; p.V_crit=13.4; p.V_floor=12.0; p.V_hyst=0.2;
    return p;
}
long v2c(double v, const BatteryParams& p){ return std::lround((v-p.batt_b)/p.batt_k); }
struct BOut{ uint8_t led; bool land; double V; };
BOut bstep(double v, const BatteryParams& p){
    BOut o; battery_step((double)v2c(v,p),&p,&o.led,&o.land,&o.V); return o;
}
} // namespace

TEST(Battery, B1_ColdStart_NoFalseTrip) {
    auto p=make_batt(); battery_reset();
    auto o=bstep(15.0,p);
    EXPECT_FALSE(o.land); EXPECT_EQ(o.led,0); EXPECT_NEAR(o.V,15.0,0.05);
}
TEST(Battery, B2_Ramp_Escalation) {
    auto p=make_batt(); battery_reset();
    const int N=4001; double w=NAN,c=NAN,f=NAN;
    for(int i=0;i<N;++i){
        double V=16.8+(11.5-16.8)*i/(N-1);
        auto o=bstep(V,p);
        if(std::isnan(w)&&o.led>=1) w=o.V;
        if(std::isnan(c)&&o.led>=2) c=o.V;
        if(std::isnan(f)&&o.land)   f=o.V;
    }
    EXPECT_NEAR(w,14.0,0.15); EXPECT_NEAR(c,13.4,0.15); EXPECT_NEAR(f,12.0,0.15);
    EXPECT_TRUE(w>c && c>f);
}
TEST(Battery, B3_Hysteresis_NoChatter) {
    auto p=make_batt(); battery_reset();
    bstep(13.95,p);
    std::mt19937 rng(0); std::uniform_real_distribution<double> U(-0.06,0.06);
    for(int i=0;i<500;++i){ auto o=bstep(13.90+U(rng),p); EXPECT_EQ(o.led,1); }
}
TEST(Battery, B4_FloorLatch_Sticky) {
    auto p=make_batt(); battery_reset();
    for(int i=0;i<50;++i)  bstep(12.5,p);
    bool land=false;
    for(int i=0;i<400;++i) land=bstep(11.5,p).land;
    EXPECT_TRUE(land);
    for(int i=0;i<300;++i) land=bstep(12.6,p).land;  // V-Erholung im Descent
    EXPECT_TRUE(land);                                // Latch haelt -> kein Grenzzyklus
}
TEST(Battery, B5_EMA_FiltersShortSag) {
    auto p=make_batt(); battery_reset();
    for(int i=0;i<200;++i) bstep(13.0,p);
    bool land=false;
    for(int i=0;i<5;++i) land=bstep(11.0,p).land;     // 50 ms Sag
    EXPECT_FALSE(land);
}
TEST(Battery, B6_CountRange) {
    auto p=make_batt();
    EXPECT_EQ(v2c(13.2,p),901);
    EXPECT_EQ(v2c(16.8,p),1147);
}
