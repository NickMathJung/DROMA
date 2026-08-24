function swarm_animation(ids, filename)
%swarm_animation  createDroneAnimation mit echten Mocap-Flugpfaden.
%   swarm_animation()        ids aus mocap.streaming_ids, Logs *_id<id>.mat
%   swarm_animation([1 3])   ids explizit
%   Zeichnet Containment-Volumen, Bezierflaeche und die realen Drohnen
%   (Pose und Lage aus Mocap). Video nach data\videos\<filename>.mp4.
%   Voraussetzung: data\swarm_ref.mat stammt vom selben Lauf wie die Logs.
arguments
    ids (1,:) double = []
    filename char = ''
end
DATA = 'C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\data';
HET  = ['C:\Users\Rakete\Documents\Drohnenversuchsstand\' ...
        'hyperbolic-2d-containment-control-with-bezier-surfaces\heterogeneous'];
if isempty(ids), ids = evalin('base', 'mocap.streaming_ids'); end
if isempty(filename), filename = char("swarm_" + strjoin(string(ids), "_")); end
T_arm = 4; % Arm-Phase der Tabellen

S = load(fullfile(DATA, 'swarm_ref.mat'), 'anim', 'ref');
n = numel(ids);
t_drones = cell(1, n); y_drones = cell(1, n);
for k = 1:n
    x  = getfield(load(fullfile(DATA, sprintf('x_id%d.mat', ids(k)))), 'x');
    q  = getfield(load(fullfile(DATA, sprintf('mocap_quat_id%d.mat', ids(k)))), 'mocap_quat');
    tf = getfield(load(fullfile(DATA, sprintf('t_flight_id%d.mat', ids(k)))), 't_flight');
    t  = tf - T_arm; % Flugzeit -> Agenten-/Tabellenzeit
    m  = t >= 0;
    % Tabellenstart vs. Logstart: erkennt ein swarm_ref.mat von einem anderen Lauf
    if k <= size(S.ref.p, 3)
        d0 = norm(squeeze(S.ref.p(1,:,k)) - x(find(m,1), :));
        if d0 > 0.2
            warning('swarm_animation:refMismatch', ...
                'swarm_ref.mat passt nicht zu den Logs (id %d: Startversatz %.2f m).', ids(k), d0);
        end
    end
    % Zustandslayout von createDroneAnimation: Spalten 1:3 Pos, 7:10 Quaternion
    t_drones{k} = t(m);
    y_drones{k} = [x(m,:), zeros(nnz(m), 3), q(m,:)];
end

addpath(HET, fullfile(HET, 'visualization'), fullfile(HET, 'drones'));
% Rotorgeometrie, X-Konfiguration
l = 0.124; alpha = 38.4;
a = l*sind(alpha); b = l*cosd(alpha);
P_drone.rotor_pos = [a -b 0; a b 0; -a b 0; -a -b 0].';
oldcd = cd(DATA); restoreCd = onCleanup(@() cd(oldcd)); %#ok<NASGU> Video -> data\videos\
% Laeufe mit Stoer-Ereignis: eigener Renderer mit Stoerpfeilen je Drohne
if isfield(S.anim, 'e_d') && max(abs(S.anim.e_d)) > 0.05
    create_drone_animation_dist(S.anim, t_drones, y_drones, P_drone, filename);
else
    createDroneAnimation(S.anim.t_sim, S.anim.y_sim, S.anim.params, S.anim.Pi, ...
                         S.anim.P_L, t_drones, y_drones, P_drone, filename);
end
end
