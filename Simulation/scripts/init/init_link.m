function link_params = init_link(quadcop, Ts_inner)
%init_link initializes the link
arguments (Input)
    quadcop struct % holding quadcopter parameters
    Ts_inner (1,1) double % base sample time
end

arguments (Output)
    link_params struct % holding parameters related to the link (for simulation) 
end

% --- Transportlatenz (ganzzahliger Delay of Ts_inner) ---
link_params.latency = 5e-3; % s
link_params.N_delay = round(link_params.latency / Ts_inner);    
assert(abs(link_params.N_delay*Ts_inner - link_params.latency) < 1e-12, ...
    'link.latency ist kein ganzzahliges Vielfaches von Ts_inner!');
 
% --- Paketverlust ---
link_params.pdrop = 0.02; % Verlustwahrscheinlichkeit
link_params.seed  = uint32(12345); % xorshift32-Seed (!= 0)
 
% --- int16-Quantisierung ---
link_params.qmax = int16(32767);
link_params.qmin = int16(-32768);
% Reihenfolge: [F_des(1) | q_des(4) | q_ref(4) | Omega_ref(3) | tau_ref(3) | q_ext(4)]
link_params.fs =  [ 40; ... % F_des [N] Hover ~10
                    1; 1; 1; 1; ... % q_des [-] Einheitsquaternion
                    1; 1; 1; 1; ... % q_ref [-] Einheitsquaternion
                    10; 10; 10; ... % Omega_ref [rad/s]  
                    2; 2; 2 ; ... % tau_ref [N*m]
                    1; 1; 1; 1]; % q_ext [-]
 
% --- Init-Paket = quantisiertes Hover-Kommando ---
cmd_hover = [ quadcop.m*quadcop.g ; ... % F_des = m g
              1; 0; 0; 0 ; ... % q_des = Identitaet
              1; 0; 0; 0 ; ... % q_ref = Identitaet
              0; 0; 0 ; ... % Omega_ref = 0
              0; 0; 0 ; ... % tau_ref = 0
              1; 0; 0; 0 ]; % q_ext = Identitaet
lsb_link      = link_params.fs / double(link_params.qmax);
link_params.pkt_init = [int16( min(max(round(cmd_hover ./ lsb_link), ...
                       double(link_params.qmin)), double(link_params.qmax)) )];
link_params.flags_init = [uint8(0); boolean(0)];
% Delay-Buffer-IC fuer Delay-Block mit InputProcessing='Elements as channels (sample based)':
% Eingang ist 19x1 -> IC muss [21, 1, N_delay] sein. Letzte Dim = Delay-Laenge.
link_params.pkt_init_delay = repmat(link_params.pkt_init, [1, 1, link_params.N_delay]);
link_params.flags_init_delay = repmat(link_params.flags_init, [1, 1, link_params.N_delay]);
end