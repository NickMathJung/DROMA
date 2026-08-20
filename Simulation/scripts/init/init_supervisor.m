function supervisor = init_supervisor(quadcop,Ts_gcs)
%init_supervisor initializes the parameters of the safety landing that runs
%   when estop is set to 1 or 2
arguments (Input)
    quadcop struct % holding quadrocopter related parameters
    Ts_gcs (1,1) double % sample time of the ground station
end

arguments (Output)
    supervisor struct 
end

% geregeltes Soft-Land
supervisor.v_sink = 0.15; % [m/s] Soll-Sinkrate

supervisor.z_ground = 0.0; % [m] z-Koordinate des Bodens

% Disarm-Marge ueber Grund: Cutoff (estop=2) bei z_est <= z_ground + margin.
supervisor.disarm_margin = 0.1; % [m]

supervisor.Ts = Ts_gcs; % [s]

% --- Info-Ausgabe ---
v_imp = sqrt(supervisor.v_sink^2 + 2*quadcop.g*supervisor.disarm_margin);
fprintf(['Supervisor: v_sink=%.2f m/s, margin=%.3f m, z_ground=%.2f m ' ...
         '-> v_impact~%.2f m/s\n'], supervisor.v_sink, supervisor.disarm_margin, ...
         supervisor.z_ground, v_imp);
assert(supervisor.v_sink > 0, 'v_sink muss > 0 sein.');
assert(supervisor.disarm_margin > 0, 'disarm_margin muss > 0 sein.');
end