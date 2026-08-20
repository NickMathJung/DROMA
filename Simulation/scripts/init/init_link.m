function link_params = init_link(quadcop, Ts_inner)
%init_link initializes the link
arguments (Input)
    quadcop struct % holding quadcopter parameters
    Ts_inner (1,1) double % base sample time
end

arguments (Output)
    link_params struct % holding parameters related to the link (for simulation)
end

% --- Transportlatenz ---
link_params.latency = 5e-3; % s
link_params.N_delay = round(link_params.latency / Ts_inner);
assert(abs(link_params.N_delay*Ts_inner - link_params.latency) < 1e-12, ...
    'link.latency ist kein ganzzahliges Vielfaches von Ts_inner!');

% --- Paketverlust ---
link_params.pdrop = 0.02; % Verlustwahrscheinlichkeit
link_params.seed  = uint32(12345); % xorshift32-Seed (!= 0)

% --- int16-Quantisierung ------------------------------------------------------
% Reihenfolge (7x1): [F_des(1) | Omega_ref(3) | tau_ref(3)]
% Quaternionen q_des/q_ref/q_ext: smallest-three (uint32)
link_params.qmax = int16(32767);
link_params.qmin = int16(-32768);
link_params.fs = [ 40; ... % F_des [N]
                   10; 10; 10; ... % Omega_ref [rad/s]
                    2;  2;  2 ]; % tau_ref [N*m]

% --- Init-Pakete (= quantisiertes Hover-Kommando) -----------------------------
% int16-Teil: [F_des; Omega_ref; tau_ref] = [m*g; 0;0;0; 0;0;0]
scal_hover = [ quadcop.m*quadcop.g; 0; 0; 0; 0; 0; 0 ];
lsb_link   = double(link_params.fs) / double(link_params.qmax);
link_params.pkt_init = int16( min(max(round(scal_hover ./ lsb_link), ...
                        double(link_params.qmin)), double(link_params.qmax)) );

% Quat-Teil: q_des = q_ref = q_ext = Identitaet -> smallest-three-Code (uint32)
id_code = pack_quat_sm3([1;0;0;0]);
link_params.q_init = [id_code; id_code; id_code];

link_params.flags_init = [uint8(0); boolean(0)]; % [estop=0; ack=false]

% --- Delay-Buffer-ICs ---------------------------------------------------------
% Je Signal ein eigener IC, letzte Dimension = Delay-Laenge N_delay
link_params.pkt_init_delay   = repmat(link_params.pkt_init,   [1, 1, link_params.N_delay]); % int16  7x1
link_params.q_init_delay     = repmat(link_params.q_init,     [1, 1, link_params.N_delay]); % uint32 3x1
link_params.flags_init_delay = repmat(link_params.flags_init, [1, 1, link_params.N_delay]); % 2x1
end