function [p_ref, v_ref, a_ref, j_ref, s_ref, yaw_ref] = ...
         safety_landcmd_flat(p_ref_in, v_ref_in, a_ref_in, j_ref_in, s_ref_in, ...
                             yaw_ref_in, batt_land, x_hat, v_sink, z_ground, Ts)
%#codegen
% safety_landcmd_flat  Onboard-Notabstieg der Flatness-Variante (Batterie leer).
%
% Sitzt im Pfad zwischen Bus_Cmd_flat-Referenzen und flatness_ctrl. Ist
% batt_land = true, ersetzt er die empfangene Trajektorie durch einen
% kontrollierten Sinkflug: x/y werden auf der aktuellen Schaetzposition
% eingefroren, z rampt mit v_sink bis z_ground. Anders als die "blinde"
% Kaskaden-Variante (safety_landcmd) hat die Flatness-MCU eine Positions-
% schaetzung (Luenberger + Mocap-Stream) -> geregelter Abstieg statt
% Schub-Reduktion.
%
% Eingaenge:
%   p_ref_in..s_ref_in : 3x1  flache Referenzen aus Bus_Cmd_flat
%   yaw_ref_in         : 3x1  [yaw; dyaw; ddyaw]
%   batt_land          : bool aus safety_battery
%   x_hat              : 3x1  Positionsschaetzung (Luenberger)
%   v_sink             : 1x1  Sinkgeschwindigkeit [m/s]     (supervisor.v_sink)
%   z_ground           : 1x1  Bodenhoehe [m]                (supervisor.z_ground)
%   Ts                 : 1x1  Abtastzeit dieses Blocks [s]
% Ausgaenge: Referenzsatz an flatness_ctrl.

persistent landing x0 y0 yaw0 zref
if isempty(landing)
    landing = false;
    x0 = 0.0; y0 = 0.0; zref = 0.0;
    yaw0 = [0.0; 0.0; 0.0];
end

if batt_land && ~landing
    landing = true;                 % Flanke: Ist-Zustand latchen
    x0   = x_hat(1);
    y0   = x_hat(2);
    zref = x_hat(3);
    yaw0 = yaw_ref_in;
end

if landing
    zref = zref - v_sink * Ts;
    if zref < z_ground
        zref = z_ground;
    end
    p_ref   = [x0; y0; zref];
    v_ref   = [0.0; 0.0; -v_sink];
    a_ref   = [0.0; 0.0; 0.0];
    j_ref   = [0.0; 0.0; 0.0];
    s_ref   = [0.0; 0.0; 0.0];
    yaw_ref = yaw0;
else
    p_ref   = p_ref_in;
    v_ref   = v_ref_in;
    a_ref   = a_ref_in;
    j_ref   = j_ref_in;
    s_ref   = s_ref_in;
    yaw_ref = yaw_ref_in;
end
end
