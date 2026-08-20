function [led, batt_land, V_filt] = safety_battery(batt_count, safety)
%#codegen
% safety_battery Onboard-Batterie-Monitor, 4S LiPo via PM06 V2.
%
% Kette: ADC-count -> V_batt -> V_filt -> led,
%        und V_filt <= V_floor latcht batt_land.
%
%   WARN/CRIT gehen auf die LED.
%   FLOOR     setzt batt_land = true und damit eine onboard blinde Landung.
% Der Latch ist permanent, es gibt kein Re-Arm.
%
% Eingaenge:
%   batt_count : ADC-Rohwert, 12 bit, 0..4095
%   safety     : struct  .batt_k .batt_b .batt_alpha .V_warn .V_crit .V_floor
%                        .V_hyst
% Ausgaenge:
%   led        : uint8  0 NORMAL / 1 WARN / 2 CRIT
%   batt_land  : bool   latched
%   V_filt     : double gefilterte Batteriespannung
%
% Hysterese: der Rueckweg Richtung NORMAL braucht zusaetzlich V_hyst.

persistent Vf state landed
if isempty(Vf)
    Vf     = safety.batt_k * double(batt_count) + safety.batt_b; % init
    state  = uint8(0);
    landed = false;
end

% --- ADC -> Spannung ---
V_raw = safety.batt_k * double(batt_count) + safety.batt_b;

% --- Tiefpass: V_filt += alpha*(V_raw - V_filt) ---
Vf = Vf + safety.batt_alpha * (V_raw - Vf);
V  = Vf;

% --- 3-stufige LED mit Hysterese ---
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