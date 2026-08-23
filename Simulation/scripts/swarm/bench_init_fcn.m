function bench_init_fcn(mocap)
%bench_init_fcn  InitFcn des Schwarm-bench: Urspruenge lesen, traj/xi0 je Drohne.
%   Alles id-adressiert: Drohne mit ids(d) = mocap.streaming_ids(d) nutzt
%   traj_id<id>, entweder die Schwarmtabelle aus init_trajectory_swarm(id) oder
%   den Wegpunkt-Fallback aus init_trajectory. Gestapelt landet alles im
%   geteilten traj (3. Dim) und in xi0_all (Spalte je Pfad).
ids = mocap.streaming_ids;
n = numel(ids);
[p0, q0] = read_swarm_origins(ids, mocap.host_ip, mocap.client_ip);
xi0_all = zeros(6, n);
for d = 1:n
    x0 = p0(:,d);
    yaw0 = atan2(2*(q0(1,d)*q0(4,d) + q0(2,d)*q0(3,d)), 1 - 2*(q0(3,d)^2 + q0(4,d)^2));
    tn = sprintf('traj_id%d', ids(d));
    has_tab = evalin('base', sprintf( ...
        'exist(''%s'',''var'') && isfield(%s,''tab_p'') && size(%s.tab_p,1) > 1', ...
        tn, tn, tn));
    if has_tab
        tp = evalin('base', tn);
        e0 = norm(x0 - tp.tab_p(1,:).');
        assert(e0 < 0.2, ['bench InitFcn: Drohne id=%d ist %.2f m vom ' ...
            'Tabellenstart entfernt - Schwarmreferenz neu erzeugen ' ...
            '(swarm_precompute).'], ids(d), e0);
        tp.yaw(:) = yaw0; % gemessenes Start-Yaw halten
        assignin('base', tn, tp);
    else
        assignin('base', tn, init_trajectory(x0, yaw0));
    end
    xi0_all(:,d) = [x0; 0; 0; 0];
end
assignin('base', 'xi0_all', xi0_all);

% Effektive Masse je Pfad fuer die Vorsteuerung
qc = evalin('base', 'quadcop');
if isfield(qc, 'm_id')
    m_all = qc.m_id(ids);
else
    m_all = repmat(qc.m, 1, n);
end
if isfield(qc, 'm_adap_id')
    m_all = m_all ./ qc.m_adap_id(ids);
end
assignin('base', 'm_all', m_all);

% traj_id* zum geteilten Multi-traj stapeln
T = evalin('base', sprintf('traj_id%d', ids(1)));
szP = size(T.P); szT = size(T.tab_p);
for d = 2:n
    td = evalin('base', sprintf('traj_id%d', ids(d)));
    assert(isequal(szP, size(td.P)) && isequal(szT, size(td.tab_p)), ...
        'bench InitFcn: traj_id%d passt nicht zu traj_id%d (Modus/Laenge).', ...
        ids(d), ids(1));
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
