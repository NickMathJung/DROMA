#!/usr/bin/env python3
# verify_quat_codegen.py -- SITL-Codegen-Treue der Quaternion-Helfer
#
# Zweck: einen eingefrorenen Testdaten-Vektor-Satz erzeugen, den sowohl MATLAB als
# auch der generierte C++/MEX-Code exakt treffen muessen. Dazu Property-Tests
# (Round-Trip, Zweig-Abdeckung, Norm/Orthonormalitaet) und eine unabhaengige
# Kreuzvalidierung der Ports gegen scipy, damit die Testdaten vertrauenswuerdig sind.
#
# Konvention (aus dcm2quat_local-Doc): R = DCM(q) = Inertial->Koerper (Aerospace),
# q = [q0 q1 q2 q3] skalar-zuerst. dcm2quat_local ist die Inverse von quat2dcm_local.
import numpy as np
import json
from scipy.spatial.transform import Rotation as Rot

# --- 1) Exakte Ports (bit-treu zur MATLAB-Quelle) ---
def dcm2quat_local(R):
    """Exakter Port der Shepperd-Methode aus pos_ctrl.m. Gibt (q, branch) zurueck.
    branch: 0=trace, 1=R11, 2=R22, 3=R33  (fuer Abdeckungsanalyse)."""
    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0:
        S = 2*np.sqrt(tr + 1.0)
        q0 = 0.25*S
        q1 = (R[1,2] - R[2,1])/S
        q2 = (R[2,0] - R[0,2])/S
        q3 = (R[0,1] - R[1,0])/S
        b = 0
    elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
        S = 2*np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        q0 = (R[1,2] - R[2,1])/S
        q1 = 0.25*S
        q2 = (R[0,1] + R[1,0])/S
        q3 = (R[0,2] + R[2,0])/S
        b = 1
    elif R[1,1] > R[2,2]:
        S = 2*np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        q0 = (R[2,0] - R[0,2])/S
        q1 = (R[0,1] + R[1,0])/S
        q2 = 0.25*S
        q3 = (R[1,2] + R[2,1])/S
        b = 2
    else:
        S = 2*np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        q0 = (R[0,1] - R[1,0])/S
        q1 = (R[0,2] + R[2,0])/S
        q2 = (R[1,2] + R[2,1])/S
        q3 = 0.25*S
        b = 3
    q = np.array([q0,q1,q2,q3])
    return q/np.linalg.norm(q), b

def quat2dcm_local(q):
    """Aerospace quat2dcm, skalar-zuerst -> DCM Inertial->Koerper.
    (Referenzformel; die Inverse davon ist nachweislich dcm2quat_local.)"""
    q0,q1,q2,q3 = q
    return np.array([
        [q0*q0+q1*q1-q2*q2-q3*q3, 2*(q1*q2+q0*q3),         2*(q1*q3-q0*q2)],
        [2*(q1*q2-q0*q3),         q0*q0-q1*q1+q2*q2-q3*q3, 2*(q2*q3+q0*q1)],
        [2*(q1*q3+q0*q2),         2*(q2*q3-q0*q1),         q0*q0-q1*q1-q2*q2+q3*q3]])

# --- 2) Unabhaengige Kreuzvalidierung gegen scipy (Ports korrekt?) ---
#    scipy: skalar-zuletzt [x,y,z,w]; as_matrix() = aktive Rotation (Koerper->Inertial)
#    -> Aerospace-DCM (Inertial->Koerper) = as_matrix().T
def sp_quat2dcm(q):     # q skalar-zuerst -> Aerospace DCM
    r = Rot.from_quat([q[1],q[2],q[3],q[0]])
    return r.as_matrix().T
def sp_dcm2quat(R):     # Aerospace DCM -> q skalar-zuerst
    r = Rot.from_matrix(R.T)
    x,y,z,w = r.as_quat()
    return np.array([w,x,y,z])

def q_close(a, b, tol=1e-9):   # bis auf Vorzeichen (Doppelueberdeckung)
    return min(np.linalg.norm(a-b), np.linalg.norm(a+b)) < tol

ok = True
def ck(name, cond):
    global ok; print(f"  [{'OK ' if cond else 'FAIL'}] {name}"); ok = ok and cond

print("="*70); print("1) Port-Kreuzvalidierung gegen scipy"); print("="*70)
rng = np.random.default_rng(20260706)
max_q2d = 0.0; max_d2q = 0.0
for _ in range(20000):
    qs = Rot.random(random_state=rng).as_quat()      # [x,y,z,w]
    q  = np.array([qs[3],qs[0],qs[1],qs[2]])         # -> skalar-zuerst
    if q[0] < 0: q = -q
    R = quat2dcm_local(q)
    max_q2d = max(max_q2d, np.max(np.abs(R - sp_quat2dcm(q))))
    qb,_ = dcm2quat_local(R)
    max_d2q = max(max_d2q, min(np.linalg.norm(qb - sp_dcm2quat(R)),
                               np.linalg.norm(qb + sp_dcm2quat(R))))
ck(f"quat2dcm_local == scipy (max |dR|={max_q2d:.2e})", max_q2d < 1e-12)
ck(f"dcm2quat_local == scipy (max |dq|={max_d2q:.2e})", max_d2q < 1e-9)

# --- 3) Property-Tests ueber Zufalls-Rotationen + adversariale Faelle ---
print("="*70); print("2) Property-Tests (Round-Trip, Norm, Zweig-Abdeckung)"); print("="*70)

def eye_close(R, tol=1e-9): return np.max(np.abs(R - np.eye(3))) < tol

# adversariale Faelle: identisch, 180 deg um x/y/z (Zweige 1/2/3), 120 deg
# um [1,1,1] (Diagonal-Fast-Gleichstand), Kleinstwinkel, q0<0.
adversarial = []
adversarial.append(("identity",        np.array([1.,0,0,0])))
adversarial.append(("180deg_x",        np.array([0.,1,0,0])))
adversarial.append(("180deg_y",        np.array([0.,0,1,0])))
adversarial.append(("180deg_z",        np.array([0.,0,0,1])))
c = np.cos(np.deg2rad(60)); s = np.sin(np.deg2rad(60))/np.sqrt(3)   # 120deg um [1,1,1]
adversarial.append(("120deg_diag",     np.array([c,s,s,s])))
adversarial.append(("120deg_diag_neg", -np.array([c,s,s,s])))       # q0<0
for ang in (1e-3, 1e-6, 1e-9):
    adversarial.append((f"small_{ang:g}_x", np.array([np.cos(ang/2),np.sin(ang/2),0,0])))
# knapp unter/ueber trace=0 (Zweigwechsel-Kante): Winkel um 2*acos-Schwelle
for deg in (89.9, 90.0, 90.1, 119.9, 120.1, 179.9):
    a = np.deg2rad(deg); axis = np.array([1.,2.,3.]); axis/=np.linalg.norm(axis)
    q = np.array([np.cos(a/2), *(np.sin(a/2)*axis)])
    adversarial.append((f"axis123_{deg}deg", q))

branch_hist = np.zeros(4, dtype=int)
max_rt_R = 0.0; max_rt_q = 0.0; max_norm_err = 0.0
worst_det = 0.0; worst_orth = 0.0

def run_case(q_in):
    global max_rt_R, max_rt_q, max_norm_err, worst_det, worst_orth
    q_in = q_in/np.linalg.norm(q_in)
    R = quat2dcm_local(q_in)
    # DCM-Gueteeigenschaften
    worst_orth = max(worst_orth, np.max(np.abs(R@R.T - np.eye(3))))
    worst_det  = max(worst_det, abs(np.linalg.det(R) - 1.0))
    q_out, b = dcm2quat_local(R)
    branch_hist[b] += 1
    max_norm_err = max(max_norm_err, abs(np.linalg.norm(q_out)-1.0))
    # q->R->q bis auf Vorzeichen
    max_rt_q = max(max_rt_q, min(np.linalg.norm(q_out-q_in), np.linalg.norm(q_out+q_in)))
    # R->q->R exakt (Doppelueberdeckung faellt hier weg)
    R2 = quat2dcm_local(q_out)
    max_rt_R = max(max_rt_R, np.max(np.abs(R2-R)))
    return R, q_out, b

# viele Zufallsfaelle
for _ in range(50000):
    qs = Rot.random(random_state=rng).as_quat()
    run_case(np.array([qs[3],qs[0],qs[1],qs[2]]))
# adversariale
adv_records = []
for name,q in adversarial:
    R,qo,b = run_case(q)
    adv_records.append((name, R, qo, b))

ck(f"q->R->q Round-Trip (bis auf Vz.), max={max_rt_q:.2e}", max_rt_q < 1e-8)
ck(f"R->q->R Round-Trip, max={max_rt_R:.2e}", max_rt_R < 1e-9)
ck(f"|q_out| == 1, max Abw={max_norm_err:.2e}", max_norm_err < 1e-12)
ck(f"DCM orthonormal, max={worst_orth:.2e}", worst_orth < 1e-12)
ck(f"det(DCM)==+1, max Abw={worst_det:.2e}", worst_det < 1e-12)
ck(f"alle 4 Shepperd-Zweige getroffen  {branch_hist.tolist()}", np.all(branch_hist > 0))

# Vorzeichen-Hemisphaere: Trace-Zweig liefert q0>=0
neg_q0_trace = 0
for _ in range(20000):
    qs = Rot.random(random_state=rng).as_quat()
    q = np.array([qs[3],qs[0],qs[1],qs[2]])
    R = quat2dcm_local(q)
    qo,b = dcm2quat_local(R)
    if b == 0 and qo[0] < -1e-15: neg_q0_trace += 1
ck("Trace-Zweig: q0>=0 (Hemisphaeren-Konsistenz)", neg_q0_trace == 0)

# --- 4) Testdaten-Vektoren erzeugen (deterministisch) -> csv + json ---
#    Deckt alle 4 Zweige + Kanten ab. Format je Zeile:
#    id, R11..R33 (row-major, 9), q0..q3 (4), branch
print("="*70); print("3) Testdaten-Vektoren schreiben"); print("="*70)
test_data = []
# adversariale zuerst (deckt Zweige 0..3 + Kanten sicher ab)
for name,R,qo,b in adv_records:
    test_data.append((name, R, qo, b))
# ein paar reproduzierbare Zufallsfaelle pro Zweig auffuellen
rng2 = np.random.default_rng(12345)
need = {0:4,1:4,2:4,3:4}
tries = 0
while any(v>0 for v in need.values()) and tries < 100000:
    tries += 1
    qs = Rot.random(random_state=rng2).as_quat()
    q = np.array([qs[3],qs[0],qs[1],qs[2]]); q/=np.linalg.norm(q)
    R = quat2dcm_local(q); qo,b = dcm2quat_local(R)
    if need[b] > 0:
        test_data.append((f"rand_b{b}_{4-need[b]}", R, qo, b)); need[b]-=1

# csv
with open("test_data_quat.csv","w") as f:
    f.write("id,R11,R12,R13,R21,R22,R23,R31,R32,R33,q0,q1,q2,q3,branch\n")
    for name,R,qo,b in test_data:
        row = [name] + [f"{x:.17g}" for x in R.reshape(-1)] + [f"{x:.17g}" for x in qo] + [str(b)]
        f.write(",".join(row)+"\n")
# json
with open("test_data_quat.json","w") as f:
    json.dump([{"id":n,"R":R.reshape(-1).tolist(),"q":qo.tolist(),"branch":int(b)}
               for n,R,qo,b in test_data], f, indent=1)

bh = np.zeros(4,int)
for *_ , b in [(g[-1],) for g in test_data]: pass
for _,_,_,b in test_data: bh[b]+=1
print(f"  {len(test_data)} Testdaten-Faelle geschrieben (test_data_quat.csv / .json)")
print(f"  Zweig-Verteilung der Testdaten: trace={bh[0]} R11={bh[1]} R22={bh[2]} R33={bh[3]}")
ck("Testdaten decken alle 4 Zweige ab", np.all(bh>0))

# --- 5) Weitere Helfer: quatMul (Hamilton), quatConj, quatRotate ---
def quatMul(a, c):     # Hamilton-Produkt r = a (x) c, skalar-zuerst
    return np.array([
        a[0]*c[0]-a[1]*c[1]-a[2]*c[2]-a[3]*c[3],
        a[0]*c[1]+a[1]*c[0]+a[2]*c[3]-a[3]*c[2],
        a[0]*c[2]-a[1]*c[3]+a[2]*c[0]+a[3]*c[1],
        a[0]*c[3]+a[1]*c[2]-a[2]*c[1]+a[3]*c[0]])
def quatConj(a):
    return np.array([a[0],-a[1],-a[2],-a[3]])
def quatRotate(q, vn):  # vb = R(q)*vn, R = Aerospace-DCM (== quat2dcm_local)
    return quat2dcm_local(q) @ vn

print("="*70); print("4) Weitere Helfer: Ports gegen scipy + Eigenschaften"); print("="*70)

# scipy-Kreuzvalidierung quatMul: welche Kompositions-Reihenfolge trifft Hamilton?
def rq(q): return Rot.from_quat([q[1],q[2],q[3],q[0]])   # skalar-zuerst -> scipy
err_ac=0.0; err_ca=0.0
Qa=[]; Qc=[]
for _ in range(20000):
    a=Rot.random(random_state=rng).as_quat(); a=np.array([a[3],a[0],a[1],a[2]])
    c=Rot.random(random_state=rng).as_quat(); c=np.array([c[3],c[0],c[1],c[2]])
    Qa.append(a); Qc.append(c)
    r=quatMul(a,c)
    sp_ac=(rq(a)*rq(c)).as_quat(); sp_ac=np.array([sp_ac[3],sp_ac[0],sp_ac[1],sp_ac[2]])
    sp_ca=(rq(c)*rq(a)).as_quat(); sp_ca=np.array([sp_ca[3],sp_ca[0],sp_ca[1],sp_ca[2]])
    err_ac=max(err_ac,min(np.linalg.norm(r-sp_ac),np.linalg.norm(r+sp_ac)))
    err_ca=max(err_ca,min(np.linalg.norm(r-sp_ca),np.linalg.norm(r+sp_ca)))
which = "R(a)*R(c)" if err_ac<err_ca else "R(c)*R(a)"
ck(f"quatMul == scipy-Komposition {which} (max |dq|={min(err_ac,err_ca):.2e})",
   min(err_ac,err_ca) < 1e-9)

# quatRotate vs scipy: vb = (as_matrix().T) @ vn
err_rot=0.0
for _ in range(20000):
    a=Rot.random(random_state=rng); q=a.as_quat(); q=np.array([q[3],q[0],q[1],q[2]])
    vn=rng.standard_normal(3)
    vb=quatRotate(q,vn); vb_sp=a.as_matrix().T @ vn
    err_rot=max(err_rot, np.linalg.norm(vb-vb_sp))
ck(f"quatRotate == scipy DCM^T*vn (max |dv|={err_rot:.2e})", err_rot < 1e-12)

# Eigenschaften
e_inv=0.0; e_id=0.0; e_assoc=0.0; e_normmul=0.0; e_rotinv=0.0; e_len=0.0; e_rotdcm=0.0
e_conjinv=0.0
for _ in range(20000):
    a=Rot.random(random_state=rng).as_quat(); a=np.array([a[3],a[0],a[1],a[2]])
    c=Rot.random(random_state=rng).as_quat(); c=np.array([c[3],c[0],c[1],c[2]])
    d=Rot.random(random_state=rng).as_quat(); d=np.array([d[3],d[0],d[1],d[2]])
    vn=rng.standard_normal(3)
    e_id   =max(e_id,   np.linalg.norm(quatMul(a,np.array([1.,0,0,0]))-a))
    e_inv  =max(e_inv,  np.linalg.norm(quatMul(a,quatConj(a))-np.array([1.,0,0,0])))
    e_conjinv=max(e_conjinv, np.linalg.norm(quatConj(quatConj(a))-a))
    e_assoc=max(e_assoc,np.linalg.norm(quatMul(quatMul(a,c),d)-quatMul(a,quatMul(c,d))))
    e_normmul=max(e_normmul, abs(np.linalg.norm(quatMul(a,c))-np.linalg.norm(a)*np.linalg.norm(c)))
    # Rotation rueckgaengig via Konjugierte
    e_rotinv=max(e_rotinv, np.linalg.norm(quatRotate(quatConj(a),quatRotate(a,vn))-vn))
    e_len  =max(e_len,  abs(np.linalg.norm(quatRotate(a,vn))-np.linalg.norm(vn)))
    e_rotdcm=max(e_rotdcm, np.linalg.norm(quatRotate(a,vn)-quat2dcm_local(a)@vn))
ck(f"quatMul: q(x)[1,0,0,0]==q  ({e_id:.2e})", e_id<1e-14)
ck(f"quatConj: q(x)conj(q)==id  ({e_inv:.2e})", e_inv<1e-13)
ck(f"quatConj involutiv  ({e_conjinv:.2e})", e_conjinv<1e-15)
ck(f"quatMul assoziativ  ({e_assoc:.2e})", e_assoc<1e-13)
ck(f"quatMul norm-multiplikativ  ({e_normmul:.2e})", e_normmul<1e-13)
ck(f"quatRotate rueckgaengig via conj  ({e_rotinv:.2e})", e_rotinv<1e-12)
ck(f"quatRotate laengentreu  ({e_len:.2e})", e_len<1e-12)
ck(f"quatRotate == quat2dcm_local*vn  ({e_rotdcm:.2e})", e_rotdcm<1e-15)

# DCM-Kompositions-Reihenfolge (fuer .m-Property dokumentieren)
e_o1=0.0; e_o2=0.0
for a,c in zip(Qa[:5000],Qc[:5000]):
    L=quat2dcm_local(quatMul(a,c))
    e_o1=max(e_o1, np.max(np.abs(L-quat2dcm_local(c)@quat2dcm_local(a))))
    e_o2=max(e_o2, np.max(np.abs(L-quat2dcm_local(a)@quat2dcm_local(c))))
dcm_order = "quat2dcm(c)*quat2dcm(a)" if e_o1<e_o2 else "quat2dcm(a)*quat2dcm(c)"
print(f"  DCM-Komposition: quat2dcm_local(quatMul(a,c)) == {dcm_order}  "
      f"(Fehler {min(e_o1,e_o2):.2e})")

# --- 6) Testdaten fuer die drei Helfer -> je eigene csv ---
print("="*70); print("5) Testdaten fuer quatMul/quatConj/quatRotate schreiben"); print("="*70)
rg = np.random.default_rng(99)
def rand_uq():
    q=Rot.random(random_state=rg).as_quat(); return np.array([q[3],q[0],q[1],q[2]])
I=np.array([1.,0,0,0])

# quatMul
gm=[]
gm.append(("id_x_id", I, I))
for i in range(6):
    a=rand_uq(); c=rand_uq(); gm.append((f"uu_{i}", a, c))
gm.append(("a_x_conj_a", (lambda a:(a,quatConj(a)))(rand_uq())[0], None))  # placeholder fix below
gm=[g for g in gm if g[2] is not None]
aa=rand_uq(); gm.append(("a_x_conja", aa, quatConj(aa)))
nn=2.0*rand_uq(); gm.append(("nonunit", nn, rand_uq()))   # reine Algebra, nicht-unit ok
with open("test_data_quatmul.csv","w") as f:
    f.write("id,a0,a1,a2,a3,c0,c1,c2,c3,r0,r1,r2,r3\n")
    for name,a,c in gm:
        r=quatMul(a,c)
        f.write(",".join([name]+[f"{x:.17g}" for x in (*a,*c,*r)])+"\n")

# quatConj
gc=[("id",I)]
for i in range(6): gc.append((f"u_{i}", rand_uq()))
gc.append(("nonunit", 3.0*rand_uq()))
with open("test_data_quatconj.csv","w") as f:
    f.write("id,a0,a1,a2,a3,r0,r1,r2,r3\n")
    for name,a in gc:
        r=quatConj(a)
        f.write(",".join([name]+[f"{x:.17g}" for x in (*a,*r)])+"\n")

# quatRotate
gr=[("id_ex", I, np.array([1.,0,0])),
    ("id_ey", I, np.array([0.,1,0])),
    ("id_ez", I, np.array([0.,0,1])),
    ("q_zero", rand_uq(), np.zeros(3))]
for i in range(6): gr.append((f"uv_{i}", rand_uq(), rg.standard_normal(3)))
with open("test_data_quatrotate.csv","w") as f:
    f.write("id,q0,q1,q2,q3,vn1,vn2,vn3,vb1,vb2,vb3\n")
    for name,q,vn in gr:
        vb=quatRotate(q,vn)
        f.write(",".join([name]+[f"{x:.17g}" for x in (*q,*vn,*vb)])+"\n")

print(f"  test_data_quatmul.csv ({len(gm)}), test_data_quatconj.csv ({len(gc)}), "
      f"test_data_quatrotate.csv ({len(gr)}) geschrieben")

print("="*70); print("GESAMT:", "ALLE GRUEN" if ok else "!!! FEHLER !!!"); print("="*70)