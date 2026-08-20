function [F, tau, dbg] = flatness_ctrl(p, v, q, omega, p_ref, v_ref, a_ref, j_ref, s_ref, yawref, k_hat, m, g, J, coefPos, coefPhi, Ts, kill, w_adapt)
%#codegen
% flatness_ctrl Flachheitsbasierter Folgeregler (exakte Zustandslinearisierung).
%   Liefert Schub + Moment [F;tau] in einem Zug aus dem vollen Zustand (p,v,R,w)
%   und der Solltrajektorie.
%
%   Konvention z-up, Schub entlang + Body-z.
%
%   Eingaenge
%     p,v      3x1  Position / Geschwindigkeit
%     q        4x1  Lage-Quaternion, Skalar zuerst
%     omega    3x1  Koerperdrehrate
%     p_ref..s_ref 3x1  flache Ausgaenge Position + Ableitungen (v,a,j,s)
%     yawref   3x1  [yaw; dyaw; ddyaw]
%     k_hat    1x1  Schub-Skalenschaetzung
%     m,g      1x1  Masse, Erdbeschleunigung
%     J        3x3  Traegheitstensor
%     coefPos  3x6  Brunovsky-Koeffizienten je Achse
%     coefPhi  1x4  Brunovsky-Koeffizienten Yaw, letzter Koeffizient = Integralgain
%     Ts       1x1  Abtastzeit des Reglers
%     kill     1x1  logical, gelatchter Not-Aus
%     w_adapt  1x1  Multipliziert nur den Zuwachs von k_hat
%   Ausgaenge
%     F        1x1  Sollschub
%     tau      3x1  Stellmoment
%     dbg      6x1  [aint(3); u_fb_raw(3)]: Integratorbeitrag und Rueckfuehrung
%                   vor der UFB_MAX-Begrenzung
%
%   Interne Zustaende: zeta1 (spez. Schub), zeta2 (dessen Ableitung)

% --- Reglerzustaende + Arming-Reset -------------------------------------------
% Solange kill anliegt, werden die Zustaende auf den Hover-Arbeitspunkt geklemmt:
% Start immer aus zeta1 = g, zeta2 = 0, eint = 0.
persistent zeta1 zeta2 eint eintPhi
if isempty(zeta1) || kill
    zeta1 = g; % Hover: spez. Schub = g
    zeta2 = 0;
    eint  = [0;0;0]; % Positions-Integrierer
    eintPhi = 0; % Yaw-Integrierer
end

eZ = [0;0;1];
R  = quat2dcm_local(q).'; % R_{n<-b}: Spalten = Body-Achsen in Inertial
w  = omega;
tw = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];

% --- aktuelle flache Groessen aus Zustand + Reglerzustand ---
a  = -g*eZ + zeta1*R(:,3);
jj = zeta2*R(:,3) + zeta1*R*tw*eZ;
projC = R(:,2).'*eZ;
yc = R(:,2) - projC*eZ;  yC = yc/norm(yc);
% Gierwinkel vorzeichenrichtig aus den Komponenten von yC
phi = atan2(-yC(1), yC(2));
xC  = [cos(phi); sin(phi); 0];
txB = cross(yC, R(:,3));
% Nenner vorzeichentreu auf |den| >= 0.3 klemmen
den  = xC.'*R(:,1);
den  = sign(den + (den == 0)) * max(abs(den), 0.3);
dphi = (norm(txB)*w(3) - w(2)*yC.'*R(:,3)) / den;

% --- Positions-Integrierer mit Anti-Windup ------------------------------------
ep   = p - p_ref;
ki   = coefPos(:,6);           
AINT_MAX = 400; % TUNING-Knopf
eint = eint + w_adapt * ep * Ts;
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

% --- Begrenzung ---------------------------------------------------------------
UFB_MAX = 700; % TUNING-Knopf [m/s^4]
u_fb_raw = u_123 - s_ref; % vor der Klemme
u_fb  = max(min(u_fb_raw, UFB_MAX), -UFB_MAX);
u_123 = s_ref + u_fb;

% Yaw-Fehler auf (-pi,pi] wickeln
ephi = atan2(sin(phi - yawref(1)), cos(phi - yawref(1)));
% --- Yaw-Integrierer mit Anti-Windup ------------------------------------------
AINT_PHI_MAX = 6; % TUNING-Knopf [rad/s^2]
eintPhi = eintPhi + w_adapt * ephi * Ts;
aintPhi = coefPhi(4) * eintPhi;
if     aintPhi >  AINT_PHI_MAX, aintPhi =  AINT_PHI_MAX; eintPhi = aintPhi/coefPhi(4);
elseif aintPhi < -AINT_PHI_MAX, aintPhi = -AINT_PHI_MAX; eintPhi = aintPhi/coefPhi(4);
end
u_4 = yawref(3) - coefPhi(2)*(dphi - yawref(2)) - coefPhi(3)*ephi - aintPhi;

% --- Winkelbeschleunigungen aus den Brunovsky-Stellgroessen ---
dwx = (-R(:,2).'*u_123 - 2*zeta2*w(1) + zeta1*w(2)*w(3))/zeta1;
dwy = ( R(:,1).'*u_123 - 2*zeta2*w(2) + zeta1*w(1)*w(3))/zeta1;
dwz = ( zeta1*(u_4*xC.'*R(:,1) + 2*dphi*w(3)*xC.'*R(:,2) - 2*dphi*w(2)*xC.'*R(:,3) ...
        - w(1)*w(2)*yC.'*R(:,2) - w(1)*w(3)*yC.'*R(:,3)) ...
        + yC.'*R(:,3)*(R(:,1).'*u_123 - 2*zeta2*w(2) - zeta1*w(1)*w(3)))/zeta1 * norm(cross(yC,R(:,3)));
dw = [dwx; dwy; dwz];

% --- Stellgroessen ---
% Schub-Skalenkorrektur ueber w_adapt geblendet
k_eff = 1 + w_adapt * (k_hat - 1);
F   = m * zeta1 / max(k_eff, 1e-3); % [N]
tau = J*dw + cross(w, J*w); % [Nm]
dbg = [aint; u_fb_raw]; % nur Telemetrie

% --- Reglerzustaende integrieren ---
z2o = zeta2;
dz2 = R(:,3).'*u_123 - 2*zeta2*R(:,3).'*R*tw*eZ + zeta1*R(:,3).'*R*(tw^2 + dw*ones(1,3))*eZ;
zeta2 = zeta2 + dz2*Ts;
zeta1 = zeta1 + z2o*Ts;
end
