function traj = init_trajectory(x0_in, yaw0_in)
%init_trajectory initializes the trajectory the quadcopter shall follow
%   with the according dwell times between adjacent trajectories
%   Optional x0_in/yaw0_in: Anfangs-Ursprung (aus Mocap). Ohne Argumente gelten
%   die fest eingetragenen Fallback-Werte (Laden ohne Motive). Die bench.slx-
%   InitFcn ruft init_trajectory(x0,yaw0) mit dem frisch gelesenen Mocap-Ursprung.
arguments (Input)
    x0_in   (:,1) double = []   % [] => Fallback-Werte benutzen
    yaw0_in (1,1) double = 0
end
arguments (Output)
    traj struct % holding parameters for the trahectory
end
auto_origin = ~isempty(x0_in);

if auto_origin % aus Mocap (bench.slx InitFcn beim Run)
    x0   = x0_in;
    yaw0 = yaw0_in;
    fprintf('S-4 Auto-Ursprung (Mocap): [%.3f %.3f %.3f] m, yaw %.1f deg\n', ...
            x0, rad2deg(yaw0));
else % Fallback ohne Motive
    x0   = [ 0;  0;  0 ];  
    yaw0 =   0; 
end

wayp1 = x0;
wayp2 = [wayp1(1); wayp1(2); wayp1(3)+1];
wayp3 = [wayp2(1)+1; wayp2(2); wayp2(3)];
wayp4 = [wayp3(1); wayp3(2)-1; wayp3(3)];
wayp5 = [wayp4(1)-1; wayp4(2); wayp4(3)];
wayp6 = [wayp5(1); wayp5(2)+1; wayp5(3)];
wayp7 = x0;
traj.P = [wayp1, wayp2, wayp3, wayp4, wayp5, wayp6, wayp7];

% Yaw konstant je Segment (N-1 Werte) [rad]
traj.yaw    = deg2rad([ 0   0   0   0  0  0]);

% Bewegungsdauer je Segment (N-1 Werte) [s]
traj.Tseg   = [ 2.0  2.0  2.0  2.0  2.0  2.0 ];

% Rastdauer je Wegpunkt (N Werte)  -- erster Wert = Anfangs-Hover
traj.Tdwell = [ 3.0  4.0  4.0  4.0  4.0  4.0  2.0 ];

% ===== S-4 Erste Start- und Landetrajektorie (Versuchsstand, Motoren AN) ==========
TEST_S4 = false;
if TEST_S4
    % if auto_origin % aus Mocap (bench.slx InitFcn beim Run)
    %     x0   = x0_in;
    %     yaw0 = yaw0_in;
    %     fprintf('S-4 Auto-Ursprung (Mocap): [%.3f %.3f %.3f] m, yaw %.1f deg\n', ...
    %             x0, rad2deg(yaw0));
    % else % Fallback ohne Motive
    %     x0   = [ 0;  0;  0 ];  
    %     yaw0 =   0; 
    % end
    z_hov = 1.0; % max Höhe [m]
    traj.P      = [ x0, x0+[0;0;z_hov], x0 ]; % Boden -> z_hov -> Boden
    traj.yaw    = [ yaw0  yaw0 ];
    traj.Tseg   = [ 3.0   3.0 ]; 
    traj.Tdwell = [ 3.0   6.0   2.0 ]; % 3s arm am Boden, 6s Hover, 2s nach Landung
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