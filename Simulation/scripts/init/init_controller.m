function controller = init_controller(quadcop)
%init_controller initializes the position and attitude controller
%   feedback gains
arguments (Input)
    quadcop struct % holding quadrocopter parameters
end

arguments (Output)
    controller struct % holding controller gains
end

% omega_n and zeta are the natural frequency and the damping ratio of the
% damped oscillator the closed loop-system is trying to achieve
omega_n_pos = 0.5*[10; 10; 12];   
omega_n_Lage = [17;17;10];   
zeta = 0.707;
controller.kR = diag(quadcop.J * omega_n_Lage.^2);
controller.kOmega =  diag(2 * zeta * quadcop.J * omega_n_Lage);
% Kein m in Kp/Kd: pos_ctrl rechnet F = m*(a_des + g - Kp*e - Kd*edot), das m
% steht also schon vor der Klammer. 
controller.Kp = diag(omega_n_pos.^2);
controller.Kd =  diag(2 * zeta * omega_n_pos);

% Ausgleich von Totzeit im System: gcu wertet die Trajektorie bei t + T_lead aus
controller.T_lead = 0.05; % [s]
end