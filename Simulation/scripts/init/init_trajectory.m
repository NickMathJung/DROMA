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
TEST_S2 = false;
if TEST_S2
    x0   = [ -0.3955;  0.099;  0.088 ];   % <-- gemessene Ruheposition [m]
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

% ===== S-4 Erste Start- und Landetrajektorie (Versuchsstand, Motoren AN) ==========
TEST_S4 = true;
if TEST_S4
    x0   = [ -0.413;  0.1358;  0.0875 ];  % Ruheposition am Boden
    yaw0  =   -0.03558;                                  % aktuelles Heading HALTEN
    z_hov = 1.0;                                           % max Höhe [m]
    traj.P      = [ x0, x0+[0;0;z_hov], x0 ];   % Boden -> 30 cm -> Boden
    traj.yaw    = [ yaw0  yaw0 ];
    traj.Tseg   = [ 3.0   3.0 ];                % ~20 cm/s hoch, ~20 cm/s runter (sanft)
    traj.Tdwell = [ 3.0   6.0   2.0 ];          % 3s arm am Boden, 6s Hover, 2s nach Landung
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