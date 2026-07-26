function traj = init_trajectory()
%init_trajectory initializes the trajectory the quadcopter shall follow
%   with the according dwell times between adjacent trajectories
arguments (Output)
    traj struct % holding parameters for the trahectory
end

traj.P = [  0   1   1   0   0 ; % x [m]
            0   0   1   1   0 ; % y [m]
            0   1   1   1   1 ]; % z [m]  

% Yaw konstant je Segment (N-1 Werte) [rad]
traj.yaw    = deg2rad([ 0   0   0   0 ]);

% Bewegungsdauer je Segment (N-1 Werte) [s]
traj.Tseg   = [ 7.0  7.0  7.0  7.0 ];

% Rastdauer je Wegpunkt (N Werte)  -- erster Wert = Anfangs-Hover
traj.Tdwell = [ 10.0  2.0  2.0  2.0  2.0 ];

% ===== S-2 Vorzeichentest Positionsregler (Tisch, BENCH, Motoren AUS) ==========
% Prueft die Kette Mocap-Position -> Positionsfehler -> Solllage -> throttle, ohne
% dass die Drohne fliegt. Ein falsches Vorzeichen hier ist Crash-Ursache C2
% (seitlicher Weglauf). Kp = m*omega_n_pos^2 ist steif: ~5 cm Versatz -> ~5 Grad
% kommandierte Kippung -> gut am throttle ablesbar, ohne zu saettigen.
%
% Vorgehen:
%   1) Modell mit Mocap-Feed + Link laufen lassen, Drohne BENCH, NICHT armen.
%   2) Ruheposition x0 (Signal x an pos_ctrl) und Yaw ablesen, unten eintragen.
%   3) TEST_S2 = true, neu initialisieren. Setpoint = x0 -> im Ruhezustand ist
%      der throttle symmetrisch (~[hover hover hover hover]).
%   4) Drohne von Hand ~5-10 cm versetzen, dabei WAAGERECHT halten (Marker frei).
%      Der Regler muss eine Kippung ZURUECK zum Sollpunkt kommandieren, also
%      GEGEN die Verschiebung. Muster ueber die Achsen-Summen ablesen.
%   5) Alle vier Horizontalrichtungen + hoch/runter durchgehen.
% Pass: die kommandierte Kippung wirkt der Verschiebung in ALLEN Richtungen
% entgegen. Laeuft sie mit, ist ein Positions-/Frame-Vorzeichen invertiert.
TEST_S2 = true;
if TEST_S2
    x0   = [ -0.7837;  0.0655;  0.118 ];   % <-- gemessene Ruheposition [m]
    yaw0 =   -0.0254;                       % <-- gemessener Yaw [rad] (aus Mocap bei TEST_S2=false!)
    % Setpoint-Versatz fuer den Vorzeichentest: EINE Achse auf +-0.05..0.10 m
    % setzen, Rest 0. Der Regler muss eine Kippung kommandieren, die die Drohne
    % ZUM versetzten Sollpunkt treibt (negative Rueckkopplung). [0;0;0] = Baseline.
    s2_offset = [ 0.0;  0.0;  -0.2 ];
    x0 = x0 + s2_offset;
    traj.P      = [x0, x0];           % ein Hover-Punkt (Differenz 0 -> keine Bewegung)
    traj.yaw    = yaw0;
    traj.Tseg   = 10.0;
    traj.Tdwell = [600.0, 10.0];      % lange am Sollpunkt halten
    fprintf('S-2 Vorzeichentest: Hover-Halt bei [%.2f %.2f %.2f] m, yaw %.1f deg\n', ...
            x0, rad2deg(yaw0));
end
% ==============================================================================

% Sanity-Checks (offline)
assert(size(traj.P,2) >= 2,                 'traj.P braucht >= 2 Wegpunkte');
assert(numel(traj.yaw)    == size(traj.P,2)-1, 'traj.yaw: N-1 Werte');
assert(numel(traj.Tseg)   == size(traj.P,2)-1, 'traj.Tseg: N-1 Werte');
assert(numel(traj.Tdwell) == size(traj.P,2),   'traj.Tdwell: N Werte');

fprintf('Trajektorie: %d Wegpunkte, Gesamtdauer %.2f s\n', ...
        size(traj.P,2), sum(traj.Tseg)+sum(traj.Tdwell));
end