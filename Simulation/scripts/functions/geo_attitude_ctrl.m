function [F, tau] = geo_attitude_ctrl(q, Omega, q_des, q_ref, Omega_ref, F_des, tau_ref, kR, kOmega)
%#codegen
% geo_attitude_ctrl  Geometrischer Lageregler auf SO(3).
%   Nach Lee/Leok/McClamroch (CDC 2010), 2-DOF-Fehlerrueckfuehrung:
%       tau = tau_ff - kR*eR - kOmega*eOmega
%
%   Es gibt zwei verschiedene Referenzen:
%     q_des : Solllage aus dem Folgeregler pos_ctrl (mit Positionsrueckfuehrung).
%             Sie bestimmt den Lagefehler e_R.
%     q_ref : nominelle Solllage aus traj_gen (reine Vorsteuerung). Sie liefert
%             den Frame, in dem Omega_ref definiert ist ([Om_ref]x = R_ref' R^T_ref),
%             und transportiert Omega_ref in den aktuellen Body-Frame.
%
%   Ein-/Ausgaenge:
%     q, Omega         : aktuelle Lage/Drehrate
%     q_des            : Solllage aus pos_ctrl
%     q_ref            : Solllage aus traj_gen 
%     Omega_ref        : Vorsteuer-Drehrate im Body-Frame aus traj_gen
%     F_des, tau_ff    : Vorsteuerung Schub/Moment aus traj_gen
%     kR, kOmega       : 3x3-Gains
%     F, tau           : Schubsollwert, Stellmoment

R    = quat2dcm_local(q)';      
Rdes = quat2dcm_local(q_des)';   
Rref = quat2dcm_local(q_ref)';    

% --- Lagefehler gegen die rueckgefuehrte Solllage R_des ---
S  = 0.5*(Rdes'*R - R'*Rdes);     % schiefsymmetrisch
eR = [S(3,2); S(1,3); S(2,1)];    % vee

% --- Drehratenfehler: Omega_ref aus R_ref-Frame in aktuellen Frame transportieren ---
eOmega = Omega - R'*Rref*Omega_ref;

% --- Stellgesetz ---
tau = tau_ref - kR*eR - kOmega*eOmega;
% Optional voll-geometrisch: 
% tau = tau + cross(Omega, J*Omega);

F = F_des;
end
