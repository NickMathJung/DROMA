function safety = init_safety(quadcop) %#ok<INUSD>
%init_safety  Parameter fuer den Onboard-Kill-Latch (Overspeed und Tilt).
arguments (Input)
    quadcop struct % derzeit ungenutzt
end
arguments (Output)
    safety struct % holding parameters for the safety function
end

% Drehraten-Schwelle je Achse, muss unter der Gyro-FSR von 8.727 rad/s liegen
safety.omega_max = 8.5; % [rad/s]

% N aufeinanderfolgende Samples
safety.debounce_N = uint16(4);

% Detektor-Modus: false = per-Achse |Omega_i|,
%                 true  = Euklidische Norm ||Omega||.
safety.use_norm = true;

% Tilt-Cutoff: Kill, wenn der Kippwinkel gegen die Vertikale tilt_max_deg ueber
% tilt_debounce_N Basistakte ueberschreitet. Verglichen wird cos(Kippwinkel).
safety.tilt_max_deg = 90; % [deg] gegen die Vertikale
safety.tilt_cos_min = cosd(safety.tilt_max_deg);
safety.tilt_debounce_N = uint16(80); % 80 Basistakte = 80 ms @1 kHz
end