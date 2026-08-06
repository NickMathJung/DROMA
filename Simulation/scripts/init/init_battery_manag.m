function safety = init_battery_manag(quadcop, safety, Ts_batt)
%init_battery_manag initializes parameters for the battery monitoring
%   (blinking LED when battery voltage drops) and the hard landing when the
%   battery voltage drops below a certain threshold
arguments (Input)
    quadcop struct % holding quadrocopter related parameters
    safety struct % already holding safety related parameters 
    Ts_batt (1,1) double % sample time for the Hyterese, so that sudden battery voltage drops don't lead to false forced landing
end

arguments (Output)
    safety struct 
end

% --- ADC / HW (PM06 V2, Teensy) ---
safety.batt_pin = 41; % Pin 41 = A17 Spannung (40/A16 = Strom).
                      % Nur Doku — im Codegen ungenutzt (Modell liest batt_count als Inport).
safety.adc_bits = 12; % analogReadResolution(12)

% V_batt = k*count + b.  k,b aus realer HW-Messung (Teensy Pin 41, 12 bit):
%   Messpunkt: batt_count = 944 <-> V_akku = 15.74 V.
%   k = 15.74/944 = 0.0166737 V/count,  b = 0 (rein ohmscher Teiler).
% Effektiver Teiler = k*4095/3.3 = 20.69:1 (Datenblatt-18.182 war zu optimistisch;
% deckt sich mit der ~21:1-Messung: 0.75 V @ 15.75 V).
safety.batt_k = 15.74/944; % Steigung = 0.0166737 V/count (HW-kalibriert)
safety.batt_b = 0.0; % offset
 
% --- Tiefpass ---
safety.batt_tau   = 0.7; % gegen Last-Einbruch + Rauschen (0.5..1)
safety.batt_alpha = 1 - exp(-Ts_batt/safety.batt_tau);  % Hysterese-Koeffizient

% --- Eigener, SCHNELLER Tiefpass fuer die Drossel-Spannungskompensation --------
% Die Kompensation thr = poly(omega)*U_ds/V soll Spannungseinbrueche aufheben,
% BEVOR sie Schub werden. V_filt (0.7 s, oben) taugt dafuer nicht -- er ist
% absichtlich traege fuer batt_land, und safety_battery laeuft obendrein nur im
% Sekundentakt: bei der im Flug gemessenen 0.46-Hz-Lastschwingung (Batterie
% corr zur Drossel -0.62, 1.7 % Spannungshub -> quadratisch ~3.4 % Schubhub)
% ist er schlicht blind. Zwei Verbraucher, zwei Filter (Nicks Vorschlag,
% 05.08.2026): batt_land behaelt die traege Hysterese, die Drossel bekommt
% diesen schnellen Pfad (mcu_flat: G_batt_k -> B_batt_b -> LP_V_thr @ Ts_inner).
% tau-Wahl: 0.1 s laesst 0.46 Hz zu 96 % durch (16 deg Verzug) und drueckt das
% ADC-Rauschen (~30 counts RMS, dominant ~45 Hz) auf ~0.3 % Drosselaequivalent.
% Die Mitkopplungsschleife thr -> I -> V-Sag -> thr hat Kreisverstaerkung
% thr*R_batt*(dI/dthr)/V ~ 0.04..0.13 -- unkritisch.
safety.tau_thr = 0.1; % [s]
 
% --- Schwellen (4S LiPo, unter Last). final auf HW bestaetigen ---
safety.V_warn = 14.0; % 3.50 V/Zelle -> LED WARN, Bediener handeln
safety.V_crit = 13.4; % 3.35 V/Zelle -> LED CRIT
safety.V_floor = 12.0; % 3.00 V/Zelle -> onboard Hard-Floor-Descent
safety.V_hyst = 0.2; % Hysterese/Recovery-Band gegen Chattering
 
% --- Harter Sinklflug ---
safety.hardfloor_thrust_frac = 0.99; % F_des = 0.98*m*g (ueberlebbare Sinkrate).
safety.m = quadcop.m;
safety.g = quadcop.g;
 
% Quervalidierung (k=0.0166737): V_akku 12.0..16.8 V -> V_pin 0.580..0.812 V
% -> count 720..1008 (~18..25 % des 12-bit-Bereichs). Schwellen in counts:
% V_warn 14.0->840, V_crit 13.4->804, V_floor 12.0->720. Keine Ueberspannung (<3.3 V).
end