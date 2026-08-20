function [q_hat, Omega_hat] = mahony_filter(imu_gyro, imu_acc, q_ext, ka, kE, Ts, q_init)
%#codegen
% mahony_filter  Expliziter Komplementaerfilter auf SO(3), zeitdiskret.
%   Nach Mahony/Hamel/Pflimlin (TAC 2008), ohne Magnetometer (km = 0). Der
%   Accel misst die spezifische Kraft; im Hover [0;0;-g], also Richtung
%   v0 = [0;0;-1].
%
%   Der Gyro-Eingang imu_gyro ist schon bias-korrigiert.
%
%   Ein-/Ausgaenge:
%     imu_gyro (3x1)  : gemessene Koerperdrehrate, bias-korrigiert
%     imu_acc  (3x1)  : gemessene spezifische Kraft im Koerperframe
%     q_ext    (4x1)  : externe Mocap-Lage
%     ka, kE          : Tilt-, Externreferenz-Gain
%     Ts              : Abtastperiode
%     q_init   (4x1)  : Startlage
%     q_hat    (4x1)  : geschaetzte Lage
%     Omega_hat(3x1)  : Drehrate

persistent q
if isempty(q)
    q = q_init; % Startlage
end

% --- 1) Innovation ---
na = norm(imu_acc);
if na > 1e-6
    v_acc = imu_acc / na; % spezifische Kraft in Körperkoordinaten
    v_hat = quatRotate(q, [0; 0; 1]); % geschaetzte Schwerkraftrichtung in Körperkoordinaten
    e_acc = cross(v_acc, v_hat); % Schätzfehler = Innovationsterm
else
    e_acc = [0; 0; 0]; % Freifall
end

% --- 2) Externe Mocap-Lage ---
nE = norm(q_ext);
if nE > 0.5
    qe    = q_ext / nE; % normieren
    q_err = quatMul(quatConj(q), qe); % Koerperinkrement, das \hat{q} -> q_ext dreht
    if q_err(1) < 0
        q_err = -q_err; % kuerzeste Drehung
    end
    e_ext = 2 * q_err(2:4); % Rotationsvektor
else
    e_ext = [0; 0; 0]; % kein gueltiges q_ext
end

% --- 3) Gewichtete Gesamt-Innovation (Koerperframe) ---
omega_mes = ka * e_acc + kE * e_ext;

% --- 5) Korrigierte Drehrate + Quaternion-Propagation ---
omega = imu_gyro + omega_mes; % Eingang der Kinematik
theta = omega * Ts;
ang   = norm(theta);
if ang > 1e-9
    norm_theta = theta / ang;
    dq = [cos(ang/2); sin(ang/2) * norm_theta];
else
    dq = [1; 0.5 * theta]; % Kleinwinkel-Naeherung
end
q = quatMul(q, dq); % Rechtsmultiplikation = Koerperinkrement
q = q / norm(q); % auf SO(3) halten

% --- 6) Ausgaenge ---
q_hat     = q;
Omega_hat = imu_gyro; % bereits bias-korrigiert
end





