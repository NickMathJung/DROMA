function [F, tau] = flatness_ctrl(p, v, q, omega, ...
                                 p_ref, v_ref, a_ref, j_ref, s_ref, yawref, ...
                                 k_hat, m, g, J, coefPos, coefPhi, Ts)
%#codegen
% flatness_ctrl Flachheitsbasierter Folgeregler (exakte Zustandslinearisierung).
%   Als Alternative zur Kaskade pos_ctrl (GCS) + geo_attitude_ctrl (MCU). 
%   Liefert Schub + Moment [F;tau] in einem Zug aus dem vollen Zustand (p,v,R,w) 
%   und der Solltrajektorie.
%
%   Konvention z-up, Schub entlang +Body-z (identisch pos_ctrl/geo_attitude_ctrl).
%   Ausgabeschnittstelle [F(N), tau(Nm)] = wie geo_attitude_ctrl -> der bestehende
%   Mixer (Gamma_inv + Throttle-Map) im mcu wird unverändert weiterverwendet.
%
%   Eingaenge
%     p,v      3x1  Position (Mocap) / Geschwindigkeit (Luenberger)
%     q        4x1  Lage-Quaternion, Skalar zuerst
%     omega    3x1  Koerperdrehrate 
%     p_ref..s_ref 3x1  flache Ausgaenge Position + Ableitungen (v,a,j,s)
%     yawref   3x1  [yaw; dyaw; ddyaw]  (aus traj_gen; dyaw=ddyaw=0 bei Segment-Yaw)
%     k_hat    1x1  Schub-Skalenschaetzung (aus flatness_khat; 1.0 wenn aus)
%     m,g      1x1  Masse, Erdbeschleunigung
%     J        3x3  Traegheitstensor
%     coefPos  3x6  Brunovsky-Koeffizienten je Achse (aus init_flatness)
%     coefPhi  1x3  Brunovsky-Koeffizienten Yaw
%     Ts       1x1  Abtastzeit des Reglers [s]
%   Ausgaenge
%     F        1x1  Sollschub [N]
%     tau      3x1  Stellmoment [Nm]
%
%   Interne Zustaende: zeta1 (spez. Schub), zeta2 (dessen Ableitung) -> die
%   dynamische Erweiterung, die den Schub zum Brunovsky-Integratorzustand macht.

persistent zeta1 zeta2 eint
if isempty(zeta1)
    zeta1 = g;          % Hover: spez. Schub = g
    zeta2 = 0;
    eint  = [0;0;0];    % Positions-Integrierer (nullt konstante Stoerungen)
end

eZ = [0;0;1]; eY = [0;1;0];
R  = quat2dcm_local(q).';        % R_{n<-b}: Spalten = Body-Achsen in Inertial
w  = omega;
tw = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];

% --- aktuelle flache Groessen aus Zustand + Reglerzustand ---
a  = -g*eZ + zeta1*R(:,3);
jj = zeta2*R(:,3) + zeta1*R*tw*eZ;
projC = R(:,2).'*eZ;
yc = R(:,2) - projC*eZ;  yC = yc/norm(yc);
phi = acos(eY.'*yC);
xC  = [cos(phi); sin(phi); 0];
txB = cross(yC, R(:,3));
dphi = (norm(txB)*w(3) - w(2)*yC.'*R(:,3)) / (xC.'*R(:,1));

% --- Positions-Integrierer mit Anti-Windup ------------------------------------
ep   = p - p_ref;
ki   = coefPos(:,6);           
AINT_MAX = 400; % TUNING-Knopf
eint = eint + ep * Ts;
aint = ki .* eint;
for kk = 1:3
    if     aint(kk) >  AINT_MAX, aint(kk) =  AINT_MAX; if ki(kk) > 0, eint(kk) = aint(kk)/ki(kk); end
    elseif aint(kk) < -AINT_MAX, aint(kk) = -AINT_MAX; if ki(kk) > 0, eint(kk) = aint(kk)/ki(kk); end
    end
end

% --- Brunovsky-Regelgesetze (Position bis Snap, Yaw bis Winkelbeschl.) ---
u_123 = s_ref ...
        - diag(coefPos(:,2))*(jj - j_ref) ...
        - diag(coefPos(:,3))*(a  - a_ref) ...
        - diag(coefPos(:,4))*(v  - v_ref) ...
        - diag(coefPos(:,5))*ep ...
        - aint;
u_4 = yawref(3) - coefPhi(2)*(dphi - yawref(2)) - coefPhi(3)*(phi - yawref(1));

% --- Winkelbeschleunigungen aus den Brunovsky-Stellgroessen ---
dwx = (-R(:,2).'*u_123 - 2*zeta2*w(1) + zeta1*w(2)*w(3))/zeta1;
dwy = ( R(:,1).'*u_123 - 2*zeta2*w(2) + zeta1*w(1)*w(3))/zeta1;
dwz = ( zeta1*(u_4*xC.'*R(:,1) + 2*dphi*w(3)*xC.'*R(:,2) - 2*dphi*w(2)*xC.'*R(:,3) ...
        - w(1)*w(2)*yC.'*R(:,2) - w(1)*w(3)*yC.'*R(:,3)) ...
        + yC.'*R(:,3)*(R(:,1).'*u_123 - 2*zeta2*w(2) - zeta1*w(1)*w(3)))/zeta1 * norm(cross(yC,R(:,3)));
dw = [dwx; dwy; dwz];

% --- Stellgroessen ---
F   = m * zeta1 / max(k_hat, 1e-3);     % [N] mit Schub-Skalenkorrektur k_hat
tau = J*dw + cross(w, J*w);             % [Nm]

% --- Reglerzustaende integrieren (expliziter Euler) ---
z2o = zeta2;
dz2 = R(:,3).'*u_123 - 2*zeta2*R(:,3).'*R*tw*eZ + zeta1*R(:,3).'*R*(tw^2 + dw*ones(1,3))*eZ;
zeta2 = zeta2 + dz2*Ts;
zeta1 = zeta1 + z2o*Ts;
end
