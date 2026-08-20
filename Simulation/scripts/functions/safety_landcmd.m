function [q_des, Omega_ref, F_cmd, tau_ref, q_ref] = ...
         safety_landcmd(F_in, q_des_in, q_ref_in, Omega_ref_in, tau_ref_in, ...
                        batt_land, safety)
%#codegen
% safety_landcmd  Onboard blinde harte Landung.
%
% Ist batt_land = true, wird der empfangene Kommandosatz durch einen Sinkflug
% ueberschrieben.
%
% Eingaenge:
%   F_in         : scalar  Schub-Kommando
%   q_des_in     : 4x1     Folgeregler-Lage
%   q_ref_in     : 4x1     Vorsteuerlage
%   Omega_ref_in : 3x1     Vorsteuer-Rate
%   tau_ref_in   : 3x1     Vorsteuer-Moment
%   batt_land    : bool    Landeanforderung
%   safety       : struct  .m .g .hardfloor_thrust_frac
% Ausgaenge: der ggf. ueberschriebene Kommandosatz.

    if batt_land
        F_cmd     = safety.hardfloor_thrust_frac * safety.m * safety.g; % < m*g
        q_des     = [1; 0; 0; 0];     
        q_ref     = [1; 0; 0; 0];
        Omega_ref = [0; 0; 0];
        tau_ref   = [0; 0; 0];
    else
        F_cmd     = F_in;
        q_des = q_des_in;
        q_ref = q_ref_in;
        Omega_ref = Omega_ref_in;
        tau_ref = tau_ref_in;
    
    end
end