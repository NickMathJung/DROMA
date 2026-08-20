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
omega_n_pos = 0.5*[10; 10; 12];   
omega_n_Lage = [17;17;10];   
zeta = 0.707;
controller.kR = diag(quadcop.J * omega_n_Lage.^2);
controller.kOmega =  diag(2 * zeta * quadcop.J * omega_n_Lage);
% Kp/Kd ohne Masse: F = m*(a_des + g - Kp*e - Kd*edot)
controller.Kp = diag(omega_n_pos.^2);
controller.Kd =  diag(2 * zeta * omega_n_pos);

% Vorhalt: die Trajektorie wird bei t + T_lead ausgewertet
controller.T_lead = 0.05; % [s]
end