function traj = init_trajectory_swarm(id, matfile, T_arm)
%init_trajectory_swarm  Schwarmtabelle fuer Drohne id in den Base-Workspace
%   Nach swarm_precompute ausfuehren, dann bench.slx normal fliegen:
%       init_trajectory_swarm(3);   % Drohne id=3 folgt ihrem Agenten
%   Schreibt traj_id<id>; die Scheibe in swarm_ref folgt aus der Position von
%   id in mocap.streaming_ids. Die Tabellenfelder aktivieren in traj_gen den
%   Lookup-Zweig; die Wegpunktfelder bleiben konsistent belegt.
arguments
    id (1,1) double = 1
    matfile char = ['C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\' ...
                    'Simulation\data\swarm_ref.mat']
    T_arm (1,1) double = 4  % anfägliche Wartezeit am Boden vor Tabellenstart
end
ids = evalin('base', 'mocap.streaming_ids');
d = find(ids == id, 1);
assert(~isempty(d), 'init_trajectory_swarm: id=%d nicht in mocap.streaming_ids %s.', ...
    id, mat2str(ids));
S = load(matfile, 'ref');
assert(d <= size(S.ref.p, 3), 'Scheibe %d fehlt in swarm_ref (neu erzeugen).', d);

tab_Ts = median(diff(S.ref.t));
nA  = round(T_arm / tab_Ts);
tp  = [repmat(S.ref.p(1,:,d), nA, 1); S.ref.p(:,:,d)];

% Feldreihenfolge wie init_trajectory (einheitliche Struktur fuer gcu-Instanzen);
% Wegpunktfelder konsistent, werden im Tabellenmodus nicht abgefahren.
traj.P      = [tp(1,:).', tp(end,:).'];
traj.yaw    = 0;
traj.Tseg   = S.ref.t(end);
traj.Tdwell = [T_arm, 0];
traj.tab_Ts = tab_Ts;
traj.tab_p  = tp;
traj.tab_v  = [zeros(nA,3); S.ref.v(:,:,d)];
traj.tab_a  = [zeros(nA,3); S.ref.a(:,:,d)];
traj.tab_j  = [zeros(nA,3); S.ref.j(:,:,d)];

assignin('base', sprintf('traj_id%d', id), traj);
fprintf(['[init_trajectory_swarm] traj_id%d: Agent (%d,%d), Arm %.0f s + T = %.1f s, ' ...
         'Start [%.2f %.2f %.2f], Ende [%.2f %.2f %.2f]\n'], id, S.ref.agents(d,1), ...
        S.ref.agents(d,2), T_arm, S.ref.t(end), traj.P(:,1), traj.P(:,2));
end
