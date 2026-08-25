function files = swarm_snapshots(t_snap, out_dir, ids)
%swarm_snapshots  Paper-Snapshots des Schwarmflugs mit Stoerpfeilen.
%   swarm_snapshots([0 6 8.9 11.5 12.3 20], 'D:\...\img\MATLAB_sim')
%   Zeichnet je Zeitpunkt Leader, Containment-Volumen, Bezierflaeche, die
%   Drohnenkoerper aus den Mocap-Logs und je Drohne den Stoerpfeil (Richtung
%   anim.dist_dir, sichtbar solange die Stoerung wirkt). Quelle:
%   data\swarm_ref.mat + Logs *_id<id>.mat. Dateinamen: exp_snapshot_NN_t*.png
arguments
    t_snap (1,:) double
    out_dir char
    ids (1,:) double = []
end
DATA = 'C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\data';
HET  = ['C:\Users\Rakete\Documents\Drohnenversuchsstand\' ...
        'hyperbolic-2d-containment-control-with-bezier-surfaces\heterogeneous'];
addpath(fullfile(HET, 'drones'));
if isempty(ids), ids = evalin('base', 'mocap.streaming_ids'); end
T_arm = 4; % Arm-Phase der Tabellen
ARROW_LEN = 0.4; % m
VD_MIN    = 0.1; % Sichtbarkeitsschwelle
ORANGE = [1, 0.5, 0];
if ~isfolder(out_dir), mkdir(out_dir); end

S = load(fullfile(DATA, 'swarm_ref.mat'), 'anim');
A = S.anim;
N = A.N;  M = A.M;  n_L = A.params.n_L;
l = size(A.P_L, 1) / 3;
dim_X = 3 * N * M;  num_pts = N * M;

% MAS-Zustand, Leader und Stoerzustand an den Snapshot-Zeiten
y_snap = interp1(A.t_sim, A.y_sim, t_snap(:));
L_pos_snap = (A.P_L * y_snap(:, 2*dim_X + (1:n_L)).').';
vd_snap = y_snap(:, 2*dim_X + n_L + 1);
n_snap = numel(t_snap);

% Drohnenposen aus den Logs
n = numel(ids);
drone_pos = zeros(3, n_snap, n);  drone_q = zeros(4, n_snap, n);
for k = 1:n
    x  = getfield(load(fullfile(DATA, sprintf('x_id%d.mat', ids(k)))), 'x');
    q  = getfield(load(fullfile(DATA, sprintf('mocap_quat_id%d.mat', ids(k)))), 'mocap_quat');
    tf = getfield(load(fullfile(DATA, sprintf('t_flight_id%d.mat', ids(k)))), 't_flight');
    t  = tf - T_arm;  m = t >= 0;
    drone_pos(:, :, k) = interp1(t(m), x(m, :), t_snap(:)).';
    drone_q  (:, :, k) = interp1(t(m), q(m, :), t_snap(:)).';
end

% Rotorgeometrie, X-Konfiguration
l_arm = 0.124; alpha = 38.4;
a = l_arm*sind(alpha); b = l_arm*cosd(alpha);
rotor_pos = [a -b 0; a b 0; -a b 0; -a -b 0].';
n_circ = 24;  r_rotor = 0.045;
theta_c = linspace(0, 2*pi, n_circ);
circle_body_xy = r_rotor * [cos(theta_c); sin(theta_c); zeros(1, n_circ)];

% Gemeinsame Achsengrenzen ueber alle Snapshots
pad = 0.25;
ax_lim = @(Lcols, Acols) [min([Lcols(:); Acols(:)]) - pad, ...
                          max([Lcols(:); Acols(:)]) + pad];
X_agt = y_snap(:, 1:dim_X);
xl = ax_lim(L_pos_snap(:, 1:3:end), X_agt(:, 1:num_pts));
yl = ax_lim(L_pos_snap(:, 2:3:end), X_agt(:, num_pts+1:2*num_pts));
zl = ax_lim(L_pos_snap(:, 3:3:end), X_agt(:, 2*num_pts+1:3*num_pts));
cube_faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

files = {};
for s = 1:n_snap
    f = figure('Color', 'w', 'Position', [100 100 900 700], 'Visible', 'off');
    ax = axes('Parent', f);
    hold(ax, 'on');  grid(ax, 'on');  axis(ax, 'equal');
    view(ax, -37.5, 20);
    xlabel(ax, '$x_1\,\mathrm{[m]}$', 'Interpreter', 'latex');
    ylabel(ax, '$x_2\,\mathrm{[m]}$', 'Interpreter', 'latex');
    zlabel(ax, '$x_3\,\mathrm{[m]}$', 'Interpreter', 'latex');
    xlim(ax, xl);  ylim(ax, yl);  zlim(ax, zl);

    L_k = reshape(L_pos_snap(s, :).', [3, l]).';
    patch(ax, 'Faces', cube_faces, 'Vertices', L_k, 'FaceColor', 'none', ...
          'EdgeColor', 'g', 'LineWidth', 1.5);
    X_ref = A.Pi * L_pos_snap(s, :).';
    surf(ax, reshape(X_ref(1:num_pts), [N, M]), ...
             reshape(X_ref(num_pts+1:2*num_pts), [N, M]), ...
             reshape(X_ref(2*num_pts+1:3*num_pts), [N, M]), ...
         'FaceColor', 'none', 'EdgeColor', 'b', 'EdgeAlpha', 0.4, 'LineWidth', 0.5);
    scatter3(ax, L_k(:,1), L_k(:,2), L_k(:,3), 100, 'r', 'filled', ...
             'MarkerEdgeColor', 'k');

    arrow = A.dist_dir * ARROW_LEN * sign(vd_snap(s));
    for d = 1:n
        p_d = drone_pos(:, s, d);
        R = quat_to_R(drone_q(:, s, d));
        tips = p_d + R * rotor_pos;
        arms = nan(3, 12);
        for i = 1:4
            arms(:, 3*(i-1) + 1) = p_d;
            arms(:, 3*(i-1) + 2) = tips(:, i);
        end
        plot3(ax, arms(1,:), arms(2,:), arms(3,:), 'k-', 'LineWidth', 2);
        R_ring = R * circle_body_xy;
        ring = nan(3, 4*(n_circ+1));
        for i = 1:4
            c0 = (i-1)*(n_circ+1) + 1;
            ring(:, c0:c0+n_circ-1) = R_ring + tips(:, i);
        end
        plot3(ax, ring(1,:), ring(2,:), ring(3,:), 'k-', 'LineWidth', 1.2);
        if abs(vd_snap(s)) > VD_MIN
            quiver3(ax, p_d(1), p_d(2), p_d(3), arrow(1), arrow(2), arrow(3), ...
                    'AutoScale', 'off', 'Color', ORANGE, 'LineWidth', 1.8, ...
                    'MaxHeadSize', 0.8);
        end
    end

    t_tag = strrep(sprintf('%.2f', t_snap(s)), '.', 'p');
    fname = fullfile(out_dir, sprintf('exp_snapshot_%02d_t%s.png', s, t_tag));
    exportgraphics(f, fname, 'Resolution', 300);
    close(f);
    files{end+1, 1} = fname; %#ok<AGROW>
    fprintf('swarm_snapshots: %s\n', fname);
end
end
