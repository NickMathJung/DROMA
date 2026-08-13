function bench_init_fcn(mocap)
%bench_init_fcn  InitFcn des Schwarm-bench: Urspruenge lesen, traj/xi0 je Drohne.
%   Tabellen-traj (trajd mit tab_p) bleibt unangetastet, der Start wird nur gegen
%   die gemessene Pose geprueft; ohne Tabelle faellt Drohne d auf die Box zurueck.
%   Vorsicht Box-Fallback: gilt er fuer BEIDE Drohnen, koennen sich die Boxen
%   schneiden -- dann nur eine Drohne einschalten.
ids = mocap.streaming_ids;
[p0, q0] = read_swarm_origins(ids, mocap.host_ip, mocap.client_ip);
for d = 1:numel(ids)
    x0 = p0(:,d);
    yaw0 = atan2(2*(q0(1,d)*q0(4,d) + q0(2,d)*q0(3,d)), 1 - 2*(q0(3,d)^2 + q0(4,d)^2));
    tn = sprintf('traj%d', d);
    has_tab = evalin('base', sprintf( ...
        'exist(''%s'',''var'') && isfield(%s,''tab_p'')', tn, tn));
    if has_tab
        tp = evalin('base', tn);
        e0 = norm(x0 - tp.tab_p(1,:).');
        assert(e0 < 0.2, ['bench InitFcn: Drohne id=%d ist %.2f m vom ' ...
            'Tabellenstart entfernt - Schwarmreferenz neu erzeugen ' ...
            '(swarm_precompute).'], ids(d), e0);
    else
        assignin('base', tn, init_trajectory(x0, yaw0));
    end
    assignin('base', sprintf('xi0%d', d), [x0; 0; 0; 0]);
end

% traj1..n zum geteilten Multi-traj stapeln (gcu waehlt per drone_idx die Scheibe)
T = evalin('base', 'traj1');
szP = size(T.P); szT = size(T.tab_p);
for d = 2:numel(ids)
    td = evalin('base', sprintf('traj%d', d));
    assert(isequal(szP, size(td.P)) && isequal(szT, size(td.tab_p)), ...
        'bench InitFcn: traj1..%d muessen denselben Modus/dieselbe Laenge haben.', d);
    T.P      = cat(3, T.P, td.P);
    T.yaw    = [T.yaw;    td.yaw];
    T.Tseg   = [T.Tseg;   td.Tseg];
    T.Tdwell = [T.Tdwell; td.Tdwell];
    T.tab_p  = cat(3, T.tab_p, td.tab_p);
    T.tab_v  = cat(3, T.tab_v, td.tab_v);
    T.tab_a  = cat(3, T.tab_a, td.tab_a);
    T.tab_j  = cat(3, T.tab_j, td.tab_j);
end
assignin('base', 'traj', T);
end
