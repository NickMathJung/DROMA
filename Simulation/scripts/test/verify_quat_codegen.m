%% verify_quat_codegen.m  -- SITL-Codegen der Quaternion-Helfer
%  SITL-Vorstufe: sicherstellen, dass die aus mcu.slx generierten C++-Helfer
%  äquivalent zu MATLAB rechnen, bevor mcu.slx als Firmware geflasht wird.
%  Prueft die Helfer (und optional die codegen-erzeugte MEX/C++-Version)
%  gegen die eingefrorenen Test-Vektoren aus verify_quat_codegen.py.
%
%  Ablauf:
%   1) golden_quat.csv laden (id, R(9), q(4), branch).
%   2) MATLAB gegen Testdatensatz:  dcm2quat_local(R) ~ +-q  und  quat2dcm_local(q) ~ R.
%   3) Property-Round-Trips ueber Zufalls-Rotationen mit den echten Helfern.
%   4) (optional) Codegen: MEX bauen und die Schritte 2-3 gegen die MEX wiederholen,
%      um zu zeigen, dass der generierte Code == Referenz (== MATLAB) ist.
%
%  Muss auf dem Pfad liegen:
%   - dcm2quat_local.m
%   - quat2dcm_local.m
clear; clc;
tol_m = 1e-15;   % MATLAB vs. Testdatensatz
tol_c = 1e-15;   % Codegen (C++) vs. Testdatensatz
pass = true;

% --- Testdatensatz laden ---
assert(isfile('C:\Users\Nick\thesis_doctoral\MAS Versuchsaufbau\Drohnen\DROMA\Simulation\scripts\test\test_data_quat2DCM.csv'), 'test_data_quat2DCM.csv fehlt (erst verify_quat_codegen.py laufen lassen).');
T   = readtable('C:\Users\Nick\thesis_doctoral\MAS Versuchsaufbau\Drohnen\DROMA\Simulation\scripts\test\test_data_quat2DCM.csv','TextType','string');
ids = T{:,1};
Rrows = T{:,2:10};         
Qref  = T{:,11:14};
Br    = T{:,15};
N = size(T,1);
fprintf('Geladen: %d Faelle, Zweig-Verteilung [%d %d %d %d]\n', N, sum(Br==0),sum(Br==1),sum(Br==2),sum(Br==3));

have_q2d = (exist('quat2dcm_local','file')==2);
if ~have_q2d
    fprintf(['HINWEIS: quat2dcm_local.m nicht auf dem Pfad -> Vorwaerts-Richtung ' ...
             '(q->R) wird uebersprungen.\n']);
end

%% ---- 2) MATLAB == Testdatensatz ----
fprintf('==== MATLAB-Helfer gegen Testdatensatz ====\n');
pass = pass & run_test(@dcm2quat_local, have_q2d, ids, Rrows, Qref, tol_m, 'MATLAB');

%% ---- 3) Property-Round-Trips ----
fprintf('==== Property-Round-Trips (MATLAB) ====\n');
pass = pass & run_roundtrips(@dcm2quat_local, have_q2d, 20000, tol_m);

%% ---- 3b) Weitere Helfer gegen Testdaten + Eigenschaften ----
fprintf('==== Weitere Helfer (MATLAB) ====\n');
pass = pass & run_quatops(@quatMul, @quatConj, @quatRotate, tol_m, 'MATLAB');

%% ---- 4) CODEGEN gegen Testdatensatz ----
codegen dcm2quat_local -args {zeros(3,3)} -o dcm2quat_local_mex -d 'C:\Users\Nick\thesis_doctoral\MAS Versuchsaufbau\Drohnen\DROMA\Simulation\scripts\codegen'
codegen quat2dcm_local -args {zeros(4,1)} -o quat2dcm_local_mex -d 'C:\Users\Nick\thesis_doctoral\MAS Versuchsaufbau\Drohnen\DROMA\Simulation\scripts\codegen'
codegen quatMul -args {zeros(4,1), zeros(4,1)} -o quatMul_mex -d 'C:\Users\Nick\thesis_doctoral\MAS Versuchsaufbau\Drohnen\DROMA\Simulation\scripts\codegen'
codegen quatConj -args {zeros(4,1)} -o quatConj_mex -d 'C:\Users\Nick\thesis_doctoral\MAS Versuchsaufbau\Drohnen\DROMA\Simulation\scripts\codegen'
codegen quatRotate -args {zeros(4,1),zeros(3,1)} -o quatRotate_mex -d 'C:\Users\Nick\thesis_doctoral\MAS Versuchsaufbau\Drohnen\DROMA\Simulation\scripts\codegen'
% 1. Add the directory to the path so MATLAB can find the newly generated MEX files
codegen_dir = 'C:\Users\Nick\thesis_doctoral\MAS Versuchsaufbau\Drohnen\DROMA\Simulation\scripts\codegen';
addpath(codegen_dir);

fprintf('==== CODEGEN-MEX gegen Testdatensatz ====\n');

% 2. Ensure you check for the correct MEX file ('dcm2quat_local_mex')
pass = pass & run_test(@dcm2quat_local_mex, ...
         exist('dcm2quat_local_mex','file')==3, ids, Rrows, Qref, tol_c, 'MEX');

% 3. Check for the MEX file again, rather than the .m file (which would return 2)
pass = pass & run_roundtrips(@dcm2quat_local_mex, ...
         exist('dcm2quat_local_mex','file')==3, 20000, tol_c);

pass = pass & run_quatops(@quatMul_mex,@quatConj_mex,@quatRotate_mex, tol_c, 'MATLAB->MEX');

fprintf('==== GESAMT: %s ====\n', tern(pass,'ALLE GRUEN','FEHLER'));

%% --- Helfer ---
function pass = run_test(d2q, have_q2d, ids, Rrows, Qref, tol, tag)
    pass = true; N = size(Rrows,1); worst_d=0; worst_f=0; bad='';
    for k=1:N
        R = reshape(Rrows(k,:),3,3).';        
        qref = Qref(k,:).';
        qk = d2q(R);
        e = min(norm(qk-qref), norm(qk+qref)); % bis auf Vorzeichen
        if e>worst_d, worst_d=e; end
        if e>tol && isempty(bad), bad=ids(k); end
        if have_q2d
            Rk = quat2dcm_local(qref);
            ef = max(abs(Rk(:)-R(:)));
            if ef>worst_f, worst_f=ef; end
            if ef>tol && isempty(bad), bad=ids(k); end
        end
    end
    pass = pass & report(sprintf('%s: dcm2quat(R) ~ +-q  (max %.2e)',tag,worst_d), worst_d<tol);
    if have_q2d
        pass = pass & report(sprintf('%s: quat2dcm(q) ~ R    (max %.2e)',tag,worst_f), worst_f<tol);
    end
    if ~isempty(bad), fprintf('     erster Ausreisser: %s\n', bad); end
end

function pass = run_roundtrips(d2q, have_q2d, M, tol)
    pass = true; rng(7); worst_R=0; worst_q=0; worst_n=0;
    if ~have_q2d
        fprintf('  (uebersprungen: quat2dcm_local fehlt)\n'); return;
    end
    for i=1:M
        q = randn(4,1); q=q/norm(q);
        R  = quat2dcm_local(q);
        qo = d2q(R);
        worst_n = max(worst_n, abs(norm(qo)-1));
        worst_q = max(worst_q, min(norm(qo-q),norm(qo+q)));
        worst_R = max(worst_R, max(abs(quat2dcm_local(qo)-R),[],'all'));
    end
    pass = pass & report(sprintf('q->R->q (bis auf Vz.), max %.2e',worst_q), worst_q<1e-8);
    pass = pass & report(sprintf('R->q->R, max %.2e',worst_R), worst_R<tol);
    pass = pass & report(sprintf('|q|==1, max %.2e',worst_n), worst_n<1e-12);
end

function s = tern(c,a,b) 
    if c 
        s=a; 
    else 
        s=b;
    end
end

function p = report(name,cond)
    fprintf('  [%s] %s\n', tern(cond,'OK ','FAIL'), name); p = cond;
end

function pass = run_quatops(qmul, qconj, qrot, tol, tag)
    pass = true;
    % quatMul gegen Golden (Quaternion-Ausgabe -> bis auf Vorzeichen)
    T=readtable('C:\Users\Nick\thesis_doctoral\MAS Versuchsaufbau\Drohnen\DROMA\Simulation\scripts\test\test_data_quatmul.csv','TextType','string'); w=0;
    for k=1:size(T,1)
        a=T{k,2:5}.'; c=T{k,6:9}.'; ref=T{k,10:13}.'; r=qmul(a,c);
        w=max(w,min(norm(r-ref),norm(r+ref)));
    end
    pass = pass & report(sprintf('%s: quatMul == Testdaten (max %.2e)',tag,w), w<tol);
    % quatConj gegen Golden (exakt, Vorzeichen fixiert)
    T=readtable('C:\Users\Nick\thesis_doctoral\MAS Versuchsaufbau\Drohnen\DROMA\Simulation\scripts\test\test_data_quatconj.csv','TextType','string'); w=0;
    for k=1:size(T,1)
        a=T{k,2:5}.'; ref=T{k,6:9}.'; w=max(w,max(abs(qconj(a)-ref)));
    end
    pass = pass & report(sprintf('%s: quatConj == Testdaten (max %.2e)',tag,w), w<tol);
    % quatRotate gegen Golden (Vektor-Ausgabe, exakt)
    T=readtable('C:\Users\Nick\thesis_doctoral\MAS Versuchsaufbau\Drohnen\DROMA\Simulation\scripts\test\test_data_quatrotate.csv','TextType','string'); w=0;
    for k=1:size(T,1)
        q=T{k,2:5}.'; vn=T{k,6:8}.'; ref=T{k,9:11}.'; w=max(w,max(abs(qrot(q,vn)-ref)));
    end
    pass = pass & report(sprintf('%s: quatRotate == Testdaten (max %.2e)',tag,w), w<tol);
    % Eigenschaften (Zufall)
    rng(11); I=[1;0;0;0]; eId=0;eInv=0;eConj=0;eAssoc=0;eNorm=0;eRot=0;eLen=0;eDcm=0;
    for i=1:20000
        a=randn(4,1);a=a/norm(a); c=randn(4,1);c=c/norm(c); d=randn(4,1);d=d/norm(d); v=randn(3,1);
        eId   =max(eId,   norm(qmul(a,I)-a));
        eInv  =max(eInv,  norm(qmul(a,qconj(a))-I));
        eConj =max(eConj, norm(qconj(qconj(a))-a));
        eAssoc=max(eAssoc,norm(qmul(qmul(a,c),d)-qmul(a,qmul(c,d))));
        eNorm =max(eNorm, abs(norm(qmul(a,c))-norm(a)*norm(c)));
        eRot  =max(eRot,  norm(qrot(qconj(a),qrot(a,v))-v));
        eLen  =max(eLen,  abs(norm(qrot(a,v))-norm(v)));
        eDcm  =max(eDcm,  norm(qrot(a,v)-quat2dcm_local(a)*v));   % Referenzbindung
    end
    pass = pass & report(sprintf('%s: quatMul id/assoc/norm (%.1e/%.1e/%.1e)',tag,eId,eAssoc,eNorm), max([eId eAssoc eNorm])<1e-12);
    pass = pass & report(sprintf('%s: quatConj involutiv & q(x)conj=id (%.1e/%.1e)',tag,eConj,eInv), max(eConj,eInv)<1e-12);
    pass = pass & report(sprintf('%s: quatRotate conj-inv/laengentreu/==DCM*vn (%.1e/%.1e/%.1e)',tag,eRot,eLen,eDcm), max([eRot eLen eDcm])<1e-11);
end