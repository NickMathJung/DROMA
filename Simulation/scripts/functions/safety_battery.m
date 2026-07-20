function [led, batt_land, V_filt] = safety_battery(batt_count, safety)
%#codegen
% safety_battery Onboard-Batterie-Monitor, 4S LiPo via PM06 V2.
%
% Kette: ADC-count -> V_batt -> V_filt -> led,
%        und V_filt <= V_floor latcht (bis Reboot) batt_land.
%
%   WARN/CRIT gehen auf die LED. Der Bediener im Raum sieht sie und loest von
%             Hand eine softe Landung aus (estop=1 per Uplink), solange noch
%             Marge da ist.
%   FLOOR     setzt batt_land = true und damit eine onboard blinde Landung
%             (safety_landcmd.m) als Backstop, falls niemand reagiert.
% Ein Kill (Overspeed/Hard-Kill) hat immer Vorrang: das nachgelagerte
% rotors_cmd=0 gewinnt.
%
% Der Latch ist permanent, es gibt kein Re-Arm. Zwei Gruende:
%   1) Ein Akkuwechsel bedeutet einen Teensy-Reboot, damit wird persistent
%      ohnehin genullt.
%   2) Der wichtigere: im Sinkflug faellt der Schub auf 0.98*m*g, es fliesst
%      weniger Strom und V erholt sich ueber den Floor. Ohne Latch wuerde
%      batt_land wieder auf null gehen, das GCS-Kommando (Hover bei m*g) kaeme
%      zurueck, die Last stiege und V sackte erneut ab: ein Grenzzyklus zwischen
%      Sinken und Schweben auf fast leerem Akku. Der Latch verhindert das, einmal
%      entschieden wird bis zum Boden gesunken.
%
% Eingaenge:
%   batt_count : ADC-Rohwert (12 bit, 0..4095). In Sim aus simulierter V-Rampe
%                (count = round((V_batt - b)/k)); auf HW analogRead(A17/Pin41).
%   safety     : struct  .batt_k .batt_b .batt_alpha .V_warn .V_crit .V_floor
%                        .V_hyst
% Ausgaenge:
%   led        : uint8  0 NORMAL / 1 WARN / 2 CRIT   (-> GPIO-Blinkmuster)
%   batt_land  : bool   latched -> safety_landcmd Hard-Floor-Override
%   V_filt     : double gefilterte Batteriespannung [V]  (dbg/logging)
%
% Hysterese: der Rueckweg Richtung NORMAL braucht zusaetzlich V_hyst, damit es
% bei Last-Sag und Rauschen nicht flattert. Der Tiefpass (tau ~0.5..1 s) glaettet
% beides und verhindert, dass ein kurzer Spannungseinbruch unter Last den Floor
% faelschlich ausloest.

persistent Vf state landed
if isempty(Vf)
    Vf     = safety.batt_k * double(batt_count) + safety.batt_b;  % init
    state  = uint8(0);
    landed = false;
end

% --- ADC -> Spannung ---
V_raw = safety.batt_k * double(batt_count) + safety.batt_b;

% --- Tiefpass: V_filt += alpha*(V_raw - V_filt) ---
Vf = Vf + safety.batt_alpha * (V_raw - Vf);
V  = Vf;

% --- 3-stufige LED mit Hysterese (Batterieanzeige) ---
% NORMAL(0) -> WARN(1) -> CRIT(2); Rueckweg braucht + V_hyst.
switch state
    case uint8(0) % NORMAL
        if V <= safety.V_warn                     
            state = uint8(1); 
        end
    case uint8(1) % WARN
        if V <= safety.V_crit                 
            state = uint8(2);
        elseif V >= safety.V_warn + safety.V_hyst
            state = uint8(0); 
        end
    otherwise % CRIT (2)
        if V >= safety.V_crit + safety.V_hyst     
            state = uint8(1); 
        end
end

% --- harte Landung ---
if V <= safety.V_floor
    landed = true;
end

led       = state;
batt_land = landed;
V_filt    = V;
end