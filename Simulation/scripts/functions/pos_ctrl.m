function [F_des, F_des_vec, q_des] = pos_ctrl(x, v, x_ref, v_ref, a_ref, yaw_ref, Kp, Kd, m, g)
%#codegen
% pos_ctrl  PD-Positionsregler.
%   Erzeugt Sollschub F, Solllage q_ref und die kommandierte
%   Inertialbeschleunigung a_cmd.
%
%   Ein-/Ausgaenge:
%     x, v            : Position/Geschwindigkeit
%     x_d, v_d, a_d   : Solltrajektorie (Pos/Geschw/Beschl, inertial)
%     yaw_ref         : Soll-Heading 3x1 [yaw; dyaw; ddyaw]
%     Kp, Kd          : 3x3-Gain-Matrizen
%     F               : Sollschubbetrag
%     q_ref           : Solllage-Quaternion
%     a_cmd           : kommandierte Inertialbeschleunigung

g_grav = [0; 0; g];

% ---- Anti-Windup-Integrator ----------
% Ki = omega_i*Kp. Die Integral-Beschleunigung wird auf +-A_INT_MAX geklemmt
% und e_int passend zurueckgerechnet.
persistent e_int

if isempty(e_int)
    e_int = [0; 0; 0]; 
end
omega_i   = 0.5; % [rad/s]
Ts        = 0.01; % [s]
A_INT_MAX = 3.0; % [m/s^2] Anti-Windup-Grenze
Ki = omega_i * Kp;

e    = x - x_ref;
edot = v - v_ref;

e_int = e_int + e * Ts;
a_int = Ki * e_int;
for k = 1:3
    if a_int(k) >  A_INT_MAX
        a_int(k) =  A_INT_MAX;  if Ki(k,k) > 0, e_int(k) = a_int(k)/Ki(k,k); end
    elseif a_int(k) < -A_INT_MAX
        a_int(k) = -A_INT_MAX;  if Ki(k,k) > 0, e_int(k) = a_int(k)/Ki(k,k); end
    end
end

% --- Soll-Schubkraft (inertial): k_thrust*m*(a_des + g_grav), a_des = a_ref + Feedback ---
k_thrust  = 1.15; % statischer Vorsteuer-Gain
F_des_vec = m*(k_thrust*(a_ref + g_grav) - Kp*e - Kd*edot - a_int);

F_des = norm(F_des_vec);

% --- Kraft -> Lage: Koerper-z entlang F_des, Heading aus yaw_ref ---
zb = F_des_vec / max(F_des, 1e-6); % gewuenschte Koerper-z in Inertial
yaw = yaw_ref(1); % yaw_ref ist 3x1 [yaw; dyaw; ddyaw]
xc = [cos(yaw); sin(yaw); 0];
yb = cross(zb, xc);  
yb = yb / max(norm(yb), 1e-6);
xb = cross(yb, zb);
Rd = [xb, yb, zb]; % R_{n<-b} (Koerper -> Inertial)
q_des = dcm2quat_local(Rd.');           

end

