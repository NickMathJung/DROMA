function [F_des, F_des_vec, q_des] = pos_ctrl(x, v, x_ref, v_ref, a_ref, yaw_ref, Kp, Kd, m, g)
%#codegen
% pos_ctrl  PD-Positionsregler.
%   Erzeugt Sollschub F, Solllage q_ref und (fuer den Beobachter) die
%   kommandierte Inertialbeschleunigung a_cmd.
%
%   Ein-/Ausgaenge:
%     x, v            : Position/Geschwindigkeit
%     x_d, v_d, a_d   : Solltrajektorie (Pos/Geschw/Beschl, inertial)
%     yaw_d           : Soll-Heading 
%     Kp, Kd          : 3x3-Gain-Matrizen
%     F               : Sollschubbetrag
%     q_ref           : Solllage-Quaternion
%     a_cmd           : kommandierte Inertialbeschleunigung (Beobachter-Eingang)

g_grav = [0; 0; g];

% --- Soll-Schubkraft (inertial): m*a_des + m*g_grav, a_des = a_d + Feedback ---
F_des_vec = m*(a_ref + g_grav - Kp*(x - x_ref) - Kd*(v - v_ref));

F_des = norm(F_des_vec);

% --- Kraft -> Lage: Koerper-z entlang F_des, Heading aus yaw_d ---
zb = F_des_vec / max(F_des, 1e-6); % gewuenschte Koerper-z in Inertial
xc = [cos(yaw_ref); sin(yaw_ref); 0];
yb = cross(zb, xc);  
yb = yb / max(norm(yb), 1e-6);
xb = cross(yb, zb);
Rd = [xb, yb, zb]; % R_{n<-b} (Koerper -> Inertial)
q_des = dcm2quat_local(Rd.');           

end

