function link_flat_params = init_link_flat(quadcop, Ts_inner)
%init_link_flat  Link-Parameter der FLATNESS-Variante (Simulation).
%   Gleiches Kanalmodell wie init_link (Latenz + Bernoulli-Verlust + int16-
%   Quantisierung), aber fuer den Bus_Cmd_flat-Inhalt:
%     int16-Teil (21x1): [mocap_pos(3); p_ref(3); v_ref(3); a_ref(3);
%                         j_ref(3); s_ref(3); yaw_ref(3)]
%     quat-Teil  (1x1):  q_ext (smallest-three, uint32)
%     flags      (2x1):  [estop; ack]
%   HINWEIS OTA: 21x int16 + uint32 + flags = ~48 B Nutzlast > 32-B-nRF-Frame.
%   Die Fluggeraet-Firmware wird das auf 2+ Frames splitten (Multi-Frame,
%   Nicks Entscheid); dieses Sim-Modell bildet Quantisierung/Latenz/Verlust
%   ab, nicht das Framing.
arguments (Input)
    quadcop struct
    Ts_inner (1,1) double
end
arguments (Output)
    link_flat_params struct
end

% --- Transportlatenz (ganzzahliger Delay of Ts_inner) ---
link_flat_params.latency = 5e-3; % s
link_flat_params.N_delay = round(link_flat_params.latency / Ts_inner);
assert(abs(link_flat_params.N_delay*Ts_inner - link_flat_params.latency) < 1e-12, ...
    'link_flat.latency ist kein ganzzahliges Vielfaches von Ts_inner!');

% --- Paketverlust ---
link_flat_params.pdrop = 0.02;
link_flat_params.seed  = uint32(24680);   % eigener Seed (!= Kaskaden-Link)

% --- int16-Quantisierung ------------------------------------------------------
% Skalen je Element; Reihenfolge wie oben. lsb = fs/qmax.
link_flat_params.qmax = int16(32767);
link_flat_params.qmin = int16(-32768);
link_flat_params.fs = [ 20; 20; 20; ...      % mocap_pos [m]    (lsb 0.6 mm)
                        20; 20; 20; ...      % p_ref     [m]
                        20; 20; 20; ...      % v_ref     [m/s]
                        50; 50; 50; ...      % a_ref     [m/s^2]
                       200;200;200; ...      % j_ref     [m/s^3]
                      2000;2000;2000; ...    % s_ref     [m/s^4]
                         4; 20; 200 ];       % yaw_ref   [rad; rad/s; rad/s^2]

% --- Init-Pakete (= Hover am Startpunkt) --------------------------------------
scal_init = [ quadcop.x0; quadcop.x0; zeros(15,1) ];   % mocap=p_ref=x0, Rest 0
lsb_link  = double(link_flat_params.fs) / double(link_flat_params.qmax);
link_flat_params.pkt_init = int16( min(max(round(scal_init ./ lsb_link), ...
                        double(link_flat_params.qmin)), double(link_flat_params.qmax)) );

% q_ext = Identitaet -> smallest-three-Code (uint32).
link_flat_params.q_init = pack_quat_sm3([1;0;0;0]);

link_flat_params.flags_init = [uint8(0); boolean(0)]; % [estop=0; ack=false]

% --- Delay-Buffer-ICs (InputProcessing='Elements as channels (sample based)') --
link_flat_params.pkt_init_delay   = repmat(link_flat_params.pkt_init,   [1, 1, link_flat_params.N_delay]);
link_flat_params.q_init_delay     = repmat(link_flat_params.q_init,     [1, 1, link_flat_params.N_delay]);
link_flat_params.flags_init_delay = repmat(link_flat_params.flags_init, [1, 1, link_flat_params.N_delay]);
end
