function [pos, yaw, ok] = read_mocap_origin(mocap, timeout_s)
%read_mocap_origin  EINE gueltige Pose des Drohnen-Rigid-Body aus Motive lesen.
%   Fuer die automatische Anfangsbedingung der Trajektorie (statt x0/yaw0 von
%   Hand aus den Mocap-Daten ablesen und in init_trajectory eintragen). Gedacht
%   fuer die InitFcn von bench.slx: beim Run einmal kurz verbinden, den aktuellen
%   Drohnen-Ursprung greifen, wieder trennen.
%
%   Nutzt denselben natnet()-Client wie MotiveMocap (Konventionen dort: Z-Up,
%   Meter, Quaternion scalar-first [w x y z]). Der NatNet-Assembly-Pfad muss
%   eingerichtet sein (Matlab\assemblypath.txt neben natnet.m) — ist er, weil
%   MotiveMocap im Flug laeuft.
%
%   Rueckgabe:
%     pos [3x1] Position [m] im Mocap-Frame (z-up)
%     yaw [1x1] Gierwinkel [rad] (ZYX, aus scalar-first Quaternion)
%     ok        true, wenn ein gueltiger Frame mit mocap.streaming_id kam
arguments (Input)
    mocap     struct
    timeout_s (1,1) double = 2.0
end
arguments (Output)
    pos (3,1) double
    yaw (1,1) double
    ok  (1,1) logical
end
    pos = [0;0;0]; yaw = 0; ok = false;

    c = natnet();
    clean = onCleanup(@() safe_disc(c));                 %#ok<NASGU> trennt immer
    if c.ConnectToNatNet(mocap.client_ip, mocap.host_ip, 'Multicast') < 1
        warning('read_mocap_origin:connect', ...
                'keine NatNet-Verbindung (Motive laeuft? Streaming an? IPs?).');
        return;
    end

    t0 = tic;
    while toc(t0) < timeout_s
        data = c.getFrame();
        if ~isempty(data) && isprop(data,'RigidBodies')
            for i = 1:data.nRigidBodies
                rb = data.RigidBodies(i);
                if rb.ID ~= mocap.streaming_id, continue; end
                % NatNet scalar-last -> scalar-first [w x y z] (wie MotiveMocap).
                q = [double(rb.qw); double(rb.qx); double(rb.qy); double(rb.qz)];
                if norm(q) < 0.5, continue; end          % untracked -> weiter warten
                q = q / norm(q);
                pos = [double(rb.x); double(rb.y); double(rb.z)];
                yaw = atan2(2*(q(1)*q(4) + q(2)*q(3)), 1 - 2*(q(3)^2 + q(4)^2));
                ok  = true;
                return;
            end
        end
        pause(0.005);
    end
    warning('read_mocap_origin:noframe', ...
            'kein gueltiger Frame fuer StreamingID=%d in %.1f s.', ...
            mocap.streaming_id, timeout_s);
end

function safe_disc(c)
    try, c.disconnect(); catch, end
end
