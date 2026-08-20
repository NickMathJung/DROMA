function [p_ref, v_ref, a_ref, j_ref, s_ref, yaw_ref] = ...
         safety_landcmd_flat(p_ref_in, v_ref_in, a_ref_in, j_ref_in, s_ref_in, ...
                             yaw_ref_in, batt_land, x_hat, v_sink, z_ground, Ts)
%#codegen
% safety_landcmd_flat  Onboard-Notabstieg der Flatness-Variante (Batterie leer).
%
% Ist batt_land = true, wird die empfangene Trajektorie durch einen
% kontrollierten Sinkflug ersetzt: x/y werden auf der aktuellen Schaetzposition
% eingefroren, z rampt mit v_sink bis z_ground.
%
% Eingaenge:
%   p_ref_in..s_ref_in : 3x1  flache Referenzen
%   yaw_ref_in         : 3x1  [yaw; dyaw; ddyaw]
%   batt_land          : bool
%   x_hat              : 3x1  Positionsschaetzung
%   v_sink             : 1x1  Sinkgeschwindigkeit
%   z_ground           : 1x1  Bodenhoehe
%   Ts                 : 1x1  Abtastzeit dieses Blocks
% Ausgaenge: Referenzsatz fuer den Regler.

persistent landing x0 y0 yaw0 zref
if isempty(landing)
    landing = false;
    x0 = 0.0; y0 = 0.0; zref = 0.0;
    yaw0 = [0.0; 0.0; 0.0];
end

if batt_land && ~landing
    landing = true; % Flanke: Ist-Zustand latchen
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
