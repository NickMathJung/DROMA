function verify_battery()
%verify_battery  Standalone-Scaffold fuer safety_battery.m (kein Modell noetig).
% Vor jedem Szenario 'clear safety_battery' (persistenter EMA-/Latch-Zustand).
% Deckt ab: Kaltstart ohne Fehltrip, LED-Eskalation 0->1->2 an den Schwellen,
% Hysterese (kein Flattern), Floor-Latch bleibt sticky (kein Auto-Release, sonst
% Sink<->Hover-Grenzzyklus), EMA filtert kurzen Last-Sag, count-Bereich.
%
% Hinweis EMA-Einschwingen: tau~0.7 s @100 Hz -> ~3*tau ≈ 2 s (200 Samples) bis
% V_filt eine Stufenaenderung trackt. Tests legen entsprechend lange Plateaus an.

m = 0.93; g = 9.81;
safety = make_safety(m, g);
v2c = @(v) round((v - safety.batt_b)/safety.batt_k);
ok = true;

% B1 ------------------------------------------- Kaltstart 15V: kein Fehltrip
clear safety_battery
[led,land,V] = safety_battery(v2c(15.0), safety);
ok = check(ok, ~land && led==0 && abs(V-15.0)<0.05, 'B1 Kaltstart 15V: kein Fehltrip, led=0');

% B2 --------------------------- langsame Rampe 16.8->11.5: Eskalation + Latch
clear safety_battery
N = 4001; Vs = linspace(16.8, 11.5, N);
w=NaN; c=NaN; f=NaN;
for i=1:N
    [led,land,V] = safety_battery(v2c(Vs(i)), safety);
    if isnan(w) && led>=1, w=V; end
    if isnan(c) && led>=2, c=V; end
    if isnan(f) && land,   f=V; end
end
ok = check(ok, abs(w-14.0)<0.15, sprintf('B2 WARN @ ~14.0V (=%.2f)', w));
ok = check(ok, abs(c-13.4)<0.15, sprintf('B2 CRIT @ ~13.4V (=%.2f)', c));
ok = check(ok, abs(f-12.0)<0.15, sprintf('B2 LAND @ ~12.0V (=%.2f)', f));
ok = check(ok, w>c && c>f,       'B2 Reihenfolge WARN>CRIT>LAND');

% B3 ------------------------------- Hysterese: V pendelt knapp unter V_warn
clear safety_battery
safety_battery(v2c(13.95), safety);            % in WARN bringen
rng(0); leds = zeros(500,1);
for i=1:500
    v = 13.90 + (rand-0.5)*0.12;
    [leds(i),~,~] = safety_battery(v2c(v), safety);
end
ok = check(ok, all(leds==1), 'B3 Hysterese: led bleibt WARN, kein Flattern');

% B4 ---------- Floor-Latch sticky: haelt trotz V-Erholung (kein Grenzzyklus)
% Modelliert genau den Descent-Fall: V faellt unter Floor (Latch), dann erholt
% sich V (weniger Last im Sinkflug); der Latch muss halten, sonst Sink<->Hover.
clear safety_battery
for i=1:50,  safety_battery(v2c(12.5), safety); end
land=false;
for i=1:400, [~,land,~] = safety_battery(v2c(11.5), safety); end   % einschwingen < floor
ok = check(ok, land, 'B4a Floor unterschritten -> land latched');
for i=1:300, [~,land,~] = safety_battery(v2c(12.6), safety); end   % Erholung im Descent
ok = check(ok, land, 'B4b sticky: land haelt trotz V-Erholung (kein Grenzzyklus)');

% B5 ---------------------------- EMA filtert kurzen Last-Sag unter Floor weg
clear safety_battery
for i=1:200, safety_battery(v2c(13.0), safety); end   % stabil ueber floor
land=false;
for i=1:5,  [~,land,V] = safety_battery(v2c(11.0), safety); end  % 50 ms Sag
ok = check(ok, ~land, sprintf('B5 50ms-Sag auf 11V: V_filt=%.2f bleibt > floor', V));

% B6 ------------------------------------------- count-Bereich vs. Handover
ok = check(ok, v2c(13.2)==901 && v2c(16.8)==1147, ...
           sprintf('B6 count %d..%d == 901..1147', v2c(13.2), v2c(16.8)));

fprintf('\n%s\n', tern(ok, '==> Alle Invarianten erfuellt', '==> Fehler: siehe FAIL'));
end

% -- Helfer ---------------------------------------------------------------
function safety = make_safety(m, g)
Ts_batt = 1/100;  tau = 0.7;
safety = struct();
safety.m = m;  safety.g = g;
safety.batt_k = 3.3*18.182/4095;     % V/count (ideal, b=0). HW-Kalibrierung noch offen.
safety.batt_b = 0.0;
safety.batt_alpha = 1 - exp(-Ts_batt/tau);
safety.V_warn  = 14.0;
safety.V_crit  = 13.4;
safety.V_floor = 12.0;
safety.V_hyst  = 0.2;
safety.hardfloor_thrust_frac = 0.92;
end

function ok = check(ok, cond, msg)
fprintf('  %s  %s\n', tern(cond,'OK  ','FAIL'), msg);
ok = ok && cond;
end

function s = tern(c,a,b)
if c, s=a; else, s=b; end
end