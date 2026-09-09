function controller = init_controller(quadcop)
%init_controller initializes the position and attitude controller
%   feedback gains
arguments (Input)
    quadcop struct % holding quadrocopter parameters
end

arguments (Output)
    controller struct % holding controller gains
end

% omega_n and zeta: natural frequency and damping ratio of the closed loop
omega_n_pos  = [4; 4; 6];
zeta_pos     = [0.8; 0.8; 0.707];
omega_n_Lage = [17; 17; 10];
zeta_Lage    = [1.0; 1.0; 0.707];
controller.kR = diag(diag(quadcop.J) .* omega_n_Lage.^2);
controller.kOmega =  diag(2 * zeta_Lage .* diag(quadcop.J) .* omega_n_Lage);
% Kp/Kd ohne Masse: F = m*(a_des + g - Kp*e - Kd*edot)
controller.Kp = diag(omega_n_pos.^2);
controller.Kd =  diag(2 * zeta_pos .* omega_n_pos);

% Vorhalt: die Trajektorie wird bei t + T_lead ausgewertet
controller.T_lead = 0.01; % [s]
end