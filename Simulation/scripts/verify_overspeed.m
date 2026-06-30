function verify_overspeed()
%VERIFY_OVERSPEED  Standalone-Scaffold fuer safety_overspeed.m (kein Modell noetig).
% Treibt die persistente MATLAB-Function mit synthetischen Sequenzen und prueft
% die §13.1-Invarianten. Vor jedem Szenario 'clear safety_overspeed', um den
% persistenten Latch-Zustand zu loeschen (sonst leckt Zustand zwischen Tests).
%
% Deckt: Entprellung (Trip exakt am N-ten Sample), Latch-Halt nach Ratenabfall,
% ack-FLANKE statt Pegel, Re-Arm-Sperre in der Luft, Hard-Kill sofort, KILL
% dominiert soft-land, per-Achse vs. norm.

safety = struct('omega_max', 10.0, 'debounce_N', 4, 'use_norm', false);
ok = true;

% S1 ------------------------------------------------------------ kein Overspeed
clear safety_overspeed
k = drive(repmat([1 1 1],20,1), zeros(20,1), false(20,1), safety);
ok = check(ok, ~any(k), 'S1 kein Overspeed -> kill stets false');

% S2 ------------------------------------------ N-1 Spikes -> kein Latch (Debounce)
clear safety_overspeed
g = [repmat([20 0 0],3,1); zeros(5,3)];
k = drive(g, zeros(8,1), false(8,1), safety);
ok = check(ok, ~any(k), 'S2 N-1 aufeinanderfolgende Spikes -> kein Latch');

% S3 ------------------------------------ N Samples -> Trip; haelt nach Ratenabfall
clear safety_overspeed
g = [repmat([20 0 0],4,1); zeros(10,3)];
[k,src] = drive(g, zeros(14,1), false(14,1), safety);
ok = check(ok, k(3)==0 && k(4)==1,        'S3 Trip exakt am N-ten (4.) Sample');
ok = check(ok, all(k(4:end)==1),          'S3 bleibt latched nach Ratenabfall');
ok = check(ok, src(4)==1,                 'S3 fault_src=1 (overspeed)');

% S4 ------------------------------- ack high WAEHREND Overspeed -> kein Re-Arm
clear safety_overspeed
g = [repmat([20 0 0],4,1); repmat([20 0 0],6,1)];
a = [false(4,1); true(6,1)];
k = drive(g, zeros(10,1), a, safety);
ok = check(ok, all(k(4:end)==1), 'S4 ack high bei laufendem Overspeed -> kein Re-Arm');

% S5 ------------------------- Rate normal, ack steigende Flanke -> Re-Arm
clear safety_overspeed
g = [repmat([20 0 0],4,1); zeros(4,3)];
a = [false(6,1); true; true];
k = drive(g, zeros(8,1), a, safety);
ok = check(ok, k(6)==1,            'S5 vor ack-Flanke noch latched');
ok = check(ok, k(7)==0,            'S5 ack-Flanke -> re-armed');
ok = check(ok, k(8)==0,            'S5 bleibt armed bei gehaltenem ack');

% S6 ----------------- ack VOR Trip dauerhaft high -> keine Flanke -> kein Auto-Re-Arm
clear safety_overspeed
g = [zeros(2,3); repmat([20 0 0],4,1); zeros(4,3)];
a = true(10,1);
k = drive(g, zeros(10,1), a, safety);
ok = check(ok, k(6)==1,            'S6 trippt trotz gehaltenem ack');
ok = check(ok, all(k(6:end)==1),   'S6 gehaltenes ack re-armt nicht (keine Flanke)');

% S7 --------------------- Hard-Kill estop==2 sofort; Re-Arm braucht estop~=2 & Flanke
clear safety_overspeed
e = [2;2;2;2;0;0];
a = [false;false;false;false;false;true];
[k,src] = drive(zeros(6,3), e, logical(a), safety);
ok = check(ok, k(1)==1 && src(1)==2, 'S7 estop=2 sofort kill, src=2');
ok = check(ok, k(4)==1,              'S7 bleibt latched solange estop=2');
ok = check(ok, k(5)==1,              'S7 estop->0 allein re-armt nicht (kein ack)');
ok = check(ok, k(6)==0,              'S7 estop=0 + ack-Flanke -> re-armed');

% S8 ----------------------------- KILL dominiert LAND: estop=1 + Overspeed -> kill
clear safety_overspeed
g = [repmat([20 0 0],4,1); zeros(3,3)];
e = ones(7,1);
[k,src] = drive(g, e, false(7,1), safety);
ok = check(ok, k(4)==1,    'S8 Overspeed killt auch bei estop=1 (soft-land)');
ok = check(ok, src(4)==1,  'S8 src=overspeed dominiert soft-land');

% S9 ------------------------------------------------- norm- vs. per-Achse-Modus
sN = safety; sN.use_norm = true;
clear safety_overspeed
k = drive(repmat([7.5 7.5 0],4,1), zeros(4,1), false(4,1), sN);  % ||.||=10.6>10
ok = check(ok, k(4)==1, 'S9 norm-Modus trippt wenn ||gyro||>omega_max');
clear safety_overspeed
k = drive(repmat([6.0 6.0 0],6,1), zeros(6,1), false(6,1), sN);  % ||.||=8.49<10
ok = check(ok, ~any(k), 'S9 norm-Modus kein Trip wenn ||gyro||<omega_max');

fprintf('\n%s\n', ternary(ok, '==> ALLE INVARIANTEN ERFUELLT', '==> FEHLER: siehe FAIL oben'));
end

% -- Helfer ---------------------------------------------------------------
function [k,src,dbg] = drive(g, estop, ack, safety)
n = size(g,1);
k = zeros(n,1); src = zeros(n,1); dbg = zeros(n,3);
for i = 1:n
    [ki, si, di] = safety_overspeed(g(i,:).', uint8(estop(i)), logical(ack(i)), safety);
    k(i)=ki; src(i)=si; dbg(i,:)=di.';
end
end

function ok = check(ok, cond, msg)
fprintf('  %s  %s\n', ternary(cond,'OK  ','FAIL'), msg);
ok = ok && cond;
end

function s = ternary(c,a,b)
if c, s=a; else, s=b; end
end