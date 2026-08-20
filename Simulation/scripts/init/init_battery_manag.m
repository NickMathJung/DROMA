function safety = init_battery_manag(quadcop, safety, Ts_batt)
%init_battery_manag initializes parameters for the battery monitoring
%   (blinking LED when battery voltage drops) and the hard landing when the
%   battery voltage drops below a certain threshold
arguments (Input)
    quadcop struct % holding quadrocopter related parameters
    safety struct % already holding safety related parameters 
    Ts_batt (1,1) double % sample time of the hysteresis
end

arguments (Output)
    safety struct 
end

% --- ADC / HW (PM06 V2, Teensy) ---
safety.batt_pin = 41; % Pin 41 = A17 Spannung
safety.adc_bits = 12; % analogReadResolution(12)

% Kennlinie des Spannungsteilers: V_batt = k*count + b
safety.batt_k = 15.74/944; % Steigung [V/count]
safety.batt_b = 0.0; % offset
 
% --- Tiefpass ---
safety.batt_tau   = 0.7; % [s] Zeitkonstante
safety.batt_alpha = 1 - exp(-Ts_batt/safety.batt_tau);  % Hysterese-Koeffizient

% --- Schneller Tiefpass fuer die Drossel-Spannungskompensation ----------------
safety.tau_thr = 0.1; % [s]
 
% --- Schwellen (4S LiPo, unter Last) ---
safety.V_warn = 14.0; % 3.50 V/Zelle -> LED WARN
safety.V_crit = 13.4; % 3.35 V/Zelle -> LED CRIT
safety.V_floor = 12.0; % 3.00 V/Zelle -> onboard Hard-Floor-Descent
safety.V_hyst = 0.2; % Hysterese-/Recovery-Band

% --- Harter Sinklflug ---
safety.hardfloor_thrust_frac = 0.99; % Anteil an m*g fuer F_des
safety.m = quadcop.m;
safety.g = quadcop.g;
end