function traj = init_trajectory_swarm(d, matfile, T_arm)
%init_trajectory_swarm  Trajektorien-Struct für die einzelnen Drohnen Schwarm
%   Nach params.m im Base-Workspace ausfuehren, dann bench.slx normal fliegen:
%       traj = init_trajectory_swarm(x);   % Drohne folgt Agent x aus swarm_ref
%   Die Tabellenfelder aktivieren in traj_gen den Lookup-Zweig; die
%   Wegpunktfelder bleiben konsistent belegt (Start/Ende der Tabelle).
arguments
    d (1,1) double = 1
    matfile char = ['C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\' ...
                    'Simulation\data\swarm_ref.mat']
    T_arm (1,1) double = 4  % anfägliche Wartezeit am Boden vor Tabellenstart 
end
S = load(matfile, 'ref');
assert(d >= 1 && d <= size(S.ref.p, 3), 'Agentenindex d ausserhalb der Tabelle.');

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

fprintf(['[init_trajectory_swarm] Agent %d (%d,%d): Arm %.0f s + T = %.1f s, ' ...
         'Start [%.2f %.2f %.2f], Ende [%.2f %.2f %.2f]\n'], d, S.ref.agents(d,1), ...
        S.ref.agents(d,2), T_arm, S.ref.t(end), traj.P(:,1), traj.P(:,2));
end
