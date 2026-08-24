function create_drone_animation_dist(anim, t_drones, y_drones, P_drone, filename)
%create_drone_animation_dist  createDroneAnimation plus Stoerpfeil je Drohne.
%   Pfeil ab Drohnenmitte in Stoerrichtung (anim.dist_dir, aus G), sichtbar
%   solange die Stoerung wirkt (v_d), feste Laenge. Video nach videos\.
arguments
    anim struct
    t_drones cell
    y_drones cell
    P_drone struct
    filename char
end
ARROW_LEN = 0.4; % m
VD_MIN    = 0.1; % Sichtbarkeitsschwelle
ORANGE = [1, 0.5, 0];

FPS      = 30;
t_dur    = min(anim.t_sim(end), min(cellfun(@(t) t(end), t_drones)));
n_frames = floor(t_dur * FPS);
t_interp = linspace(0, t_dur, n_frames);
y_interp = interp1(anim.t_sim, anim.y_sim, t_interp);

N   = anim.N;  M = anim.M;
n_L = anim.params.n_L;
l   = size(anim.P_L, 1) / 3;
n_drones = length(t_drones);
dim_X    = 3 * N * M;
num_pts  = N * M;

drone_pos = zeros(3, n_frames, n_drones);
drone_q   = zeros(4, n_frames, n_drones);
for d = 1:n_drones
    y_d = interp1(t_drones{d}, y_drones{d}, t_interp);
    drone_pos(:, :, d) = y_d(:, 1:3).';
    drone_q  (:, :, d) = y_d(:, 7:10).';
end

idx_vL = 2 * dim_X + 1;
v_L_hist = y_interp(:, idx_vL : idx_vL + n_L - 1);
L_pos_hist = (anim.P_L * v_L_hist.').';
vd_interp = y_interp(:, 2 * dim_X + n_L + 1); % Stoerzustand v_d

% Rotorkreis-Schablone in Koerper-xy
n_circ = 24;
r_rotor = 0.045;
theta_c = linspace(0, 2*pi, n_circ);
circle_body_xy = r_rotor * [cos(theta_c); sin(theta_c); zeros(1, n_circ)];

if ~isfolder('videos'), mkdir('videos'); end
full_filename = fullfile('videos', [filename '.mp4']);
v_writer = VideoWriter(full_filename, 'MPEG-4');
v_writer.FrameRate = FPS;
v_writer.Quality   = 95;
open(v_writer);

f = figure('Name', 'Drone Swarm Animation', 'Color', 'w', ...
           'Position', [100 100 1200 900]);
ax = axes('Parent', f);
hold(ax, 'on');  grid(ax, 'on');  axis(ax, 'equal');  view(ax, 3);
xlabel(ax, 'X [m]');  ylabel(ax, 'Y [m]');  zlabel(ax, 'Z [m]');

h_leaders = scatter3(ax, nan, nan, nan, 100, 'r', 'filled', ...
                     'MarkerEdgeColor', 'k', 'DisplayName', 'Leaders');
h_hull    = patch  (ax, 'Faces', [], 'Vertices', [], ...
                    'FaceColor', 'none', 'EdgeColor', 'g', ...
                    'LineWidth', 1.5, 'DisplayName', 'Containment');
h_surf    = surf   (ax, nan(N, M), nan(N, M), nan(N, M), ...
                    'FaceColor', 'none', 'EdgeColor', 'b', ...
                    'EdgeAlpha', 0.4, 'LineWidth', 0.5, ...
                    'DisplayName', 'Bezier surface');

h_arms   = gobjects(n_drones, 1);
h_rotors = gobjects(n_drones, 1);
h_dist   = gobjects(n_drones, 1);
for d = 1:n_drones
    h_arms(d)   = plot3(ax, nan, nan, nan, 'k-', 'LineWidth', 2);
    h_rotors(d) = plot3(ax, nan, nan, nan, 'k-', 'LineWidth', 1.2);
    h_dist(d)   = quiver3(ax, nan, nan, nan, nan, nan, nan, ...
                          'AutoScale', 'off', 'Color', ORANGE, ...
                          'LineWidth', 2, 'MaxHeadSize', 0.8);
end

h_title = title(ax, 't = 0.00 s');
legend([h_leaders, h_hull, h_surf, h_dist(1)], ...
       {'Leaders', 'Containment', 'Bezier surface', 'Disturbance'}, ...
       'Location', 'northeast');

pad = 1;
lx = L_pos_hist(:, 1:3:end);  ly = L_pos_hist(:, 2:3:end);
lz = L_pos_hist(:, 3:3:end);
xlim(ax, [min(lx, [], 'all') - pad, max(lx, [], 'all') + pad]);
ylim(ax, [min(ly, [], 'all') - pad, max(ly, [], 'all') + pad]);
zlim(ax, [min(lz, [], 'all') - pad, max(lz, [], 'all') + pad]);

fprintf('Rendering %d frames -> %s ...\n', n_frames, full_filename);
wb = waitbar(0, 'Rendering drone-swarm animation...');

for k = 1:n_frames
    L_k = L_pos_hist(k, :).';
    L_k_reshaped = reshape(L_k, [3, l]).';

    X_ref = anim.Pi * L_k;
    Xg = reshape(X_ref(1           :   num_pts), [N, M]);
    Yg = reshape(X_ref(  num_pts+1 : 2*num_pts), [N, M]);
    Zg = reshape(X_ref(2*num_pts+1 : 3*num_pts), [N, M]);

    set(h_leaders, 'XData', L_k_reshaped(:, 1), ...
                   'YData', L_k_reshaped(:, 2), ...
                   'ZData', L_k_reshaped(:, 3));

    if l == 8
        cube_faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
        set(h_hull, 'Faces', cube_faces, 'Vertices', L_k_reshaped);
    end

    set(h_surf, 'XData', Xg, 'YData', Yg, 'ZData', Zg);

    arrow = anim.dist_dir * ARROW_LEN * sign(vd_interp(k));
    show_arrow = abs(vd_interp(k)) > VD_MIN;

    for d = 1:n_drones
        p_d = drone_pos(:, k, d);
        q_d = drone_q  (:, k, d);
        R   = quat_to_R(q_d);
        tips = p_d + R * P_drone.rotor_pos;

        arms = nan(3, 3 * 4);
        for i = 1:4
            arms(:, 3*(i-1) + 1) = p_d;
            arms(:, 3*(i-1) + 2) = tips(:, i);
        end
        set(h_arms(d), 'XData', arms(1, :), 'YData', arms(2, :), 'ZData', arms(3, :));

        R_ring = R * circle_body_xy;
        ring = nan(3, 4 * (n_circ + 1));
        for i = 1:4
            col0 = (i - 1) * (n_circ + 1) + 1;
            ring(:, col0 : col0 + n_circ - 1) = R_ring + tips(:, i);
        end
        set(h_rotors(d), 'XData', ring(1, :), 'YData', ring(2, :), 'ZData', ring(3, :));

        if show_arrow
            set(h_dist(d), 'XData', p_d(1), 'YData', p_d(2), 'ZData', p_d(3), ...
                           'UData', arrow(1), 'VData', arrow(2), 'WData', arrow(3));
        else
            set(h_dist(d), 'XData', nan, 'YData', nan, 'ZData', nan, ...
                           'UData', nan, 'VData', nan, 'WData', nan);
        end
    end

    set(h_title, 'String', sprintf('Time: %.2f s', t_interp(k)));

    writeVideo(v_writer, getframe(f));
    if mod(k, 10) == 0, waitbar(k / n_frames, wb); end
end

close(wb);   close(v_writer);   close(f);
fprintf('Animation saved to %s\n', full_filename);
end
