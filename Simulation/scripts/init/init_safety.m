function safety = init_safety(quadcop) %#ok<INUSD>
%init_safety  Parameter fuer safety_overspeed (Onboard-Kill-Latch).
arguments (Input)
    quadcop struct % derzeit ungenutzt (war Hover-Schub fuer den verworfenen
                   % Idle-Interlock); Signatur bleibt fuer die Aufrufer stabil.
end
arguments (Output)
    safety struct % holding parameters for the safety function
end

% 8.5 rad/s (~487 deg/s) per Achse ggf. per-Achse differenzieren
safety.omega_max = 8.5; % [rad/s]

% N aufeinanderfolgende Samples gegen Gyro-Spikes
safety.debounce_N = uint16(4);

% Detektor-Modus: false = per-Achse |Omega_i| (empfohlen, achsselektiv),
%                 true  = Euklidische Norm ||Omega||.
safety.use_norm = true;

% Tilt-Cutoff: kippt der Quadrokopter mehr als tilt_max_deg gegen die Vertikale
% und haelt das ueber tilt_debounce_N Basistakte (@1 kHz also ms) an, latcht der
% Kill ebenfalls. Der Vergleich laeuft ueber cos(Kippwinkel), daher hier der
% vorberechnete Cosinus (groesserer Winkel = kleinerer Cosinus).
safety.tilt_max_deg     = 80;                      % [deg] gegen die Vertikale
safety.tilt_cos_min     = cosd(safety.tilt_max_deg);
safety.tilt_debounce_N  = uint16(80);              % 80 Basistakte = 80 ms @1 kHz

% rearm_idle_frac/F_rearm_idle (Arming-Idle-Interlock) sind entfallen; die
% Begruendung steht im Schlusskommentar von safety_overspeed.m.
end