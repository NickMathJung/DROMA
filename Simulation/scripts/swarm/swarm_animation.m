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
T_arm = 4;  % Arm-Phase der Tabellen (init_trajectory_swarm-Default)

S = load(fullfile(DATA, 'swarm_ref.mat'), 'anim');
n = numel(ids);
t_drones = cell(1, n); y_drones = cell(1, n);
for k = 1:n
    x  = getfield(load(fullfile(DATA, sprintf('x_id%d.mat', ids(k)))), 'x');
    q  = getfield(load(fullfile(DATA, sprintf('mocap_quat_id%d.mat', ids(k)))), 'mocap_quat');
    tf = getfield(load(fullfile(DATA, sprintf('t_flight_id%d.mat', ids(k)))), 't_flight');
    t  = tf - T_arm;                     % Flugzeit -> Agenten-/Tabellenzeit
    m  = t >= 0;
    % Zustandslayout von createDroneAnimation: Spalten 1:3 Pos, 7:10 Quaternion
    t_drones{k} = t(m);
    y_drones{k} = [x(m,:), zeros(nnz(m), 3), q(m,:)];
end

addpath(HET, fullfile(HET, 'visualization'), fullfile(HET, 'drones'));
% Nur die Rotorgeometrie wird gebraucht (X-Konfiguration wie init_quadcop;
% quadrotor_params hat einen c_tau-Laufzeitfehler und bleibt unangetastet).
l = 0.124; alpha = 38.4;
a = l*sind(alpha); b = l*cosd(alpha);
P_drone.rotor_pos = [a -b 0; a b 0; -a b 0; -a -b 0].';
oldcd = cd(DATA); restoreCd = onCleanup(@() cd(oldcd)); %#ok<NASGU> Video -> data\videos\
createDroneAnimation(S.anim.t_sim, S.anim.y_sim, S.anim.params, S.anim.Pi, ...
                     S.anim.P_L, t_drones, y_drones, P_drone, filename);
end
