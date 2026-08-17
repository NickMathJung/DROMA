function [p0, q0] = read_swarm_origins(ids, host_ip, client_ip, timeout_s)
%read_swarm_origins  Startposen mehrerer Rigid Bodies aus einem NatNet-Frame.
%   [p0, q0] = read_swarm_origins([1 2]) liefert p0 (3xn, [m]) und q0 (4xn,
%   scalar-first) der Bodies mit den Streaming-IDs ids. Pollt bis ALLE Bodies
%   in einem Frame gueltig sind (fuer die Agenten-Startpositionen in
%   swarm_precompute muessen beide Drohnen gleichzeitig getrackt sein).
%   Voraussetzungen wie MotiveMocap.m: Motive streamt mit Up Axis = Z,
%   setup_motive_path() ist gelaufen (assemblypath.txt).
arguments
    ids (1,:) double = [1 2]
    host_ip   char = '127.0.0.1'
    client_ip char = '127.0.0.1'
    timeout_s (1,1) double = 5
end

n  = numel(ids);
p0 = nan(3, n);
q0 = nan(4, n);

client = natnet();
ok = client.ConnectToNatNet(client_ip, host_ip, 'Multicast');
assert(ok >= 1, 'read_swarm_origins: NatNet-Verbindung fehlgeschlagen.');
cleanup = onCleanup(@() client.disconnect());

t0 = tic;
while toc(t0) < timeout_s
    data = client.getFrame();
    if isempty(data) || ~isprop(data, 'RigidBodies'), continue; end
    got = false(1, n);
    for i = 1:data.nRigidBodies
        rb = data.RigidBodies(i);
        k = find(ids == rb.ID);   % alle Spalten dieser id (doppelt = Solobetrieb)
        if isempty(k), continue; end
        p = [double(rb.x); double(rb.y); double(rb.z)];
        q = [double(rb.qw); double(rb.qx); double(rb.qy); double(rb.qz)];
        if norm(q) < 0.5 || any(~isfinite(p)), continue; end
        p0(:,k) = repmat(p, 1, numel(k));
        q0(:,k) = repmat(q / norm(q), 1, numel(k));
        got(k) = true;
    end
    if all(got)
        fprintf('[read_swarm_origins] Posen: %s\n', mat2str(round(p0, 3)));
        return;
    end
    pause(0.05);
end
error('read_swarm_origins: nicht alle IDs %s innerhalb %.1f s getrackt.', ...
      mat2str(ids), timeout_s);
end
