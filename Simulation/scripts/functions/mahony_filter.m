function [q_hat, Omega_hat] = mahony_filter(imu_gyro, imu_acc, q_ext, ka, kE, Ts, q_init)
%#codegen
% mahony_filter  Expliziter Komplementaerfilter auf SO(3), zeitdiskret.
%   Nach Mahony/Hamel/Pflimlin (TAC 2008), ohne Magnetometer (km = 0). Der
%   Accel misst die spezifische Kraft; im Hover [0;0;-g], also Richtung
%   v0 = [0;0;-1].
%
%   Der Gyro-Eingang imu_gyro ist schon bias-korrigiert. Der Bias wird
%   ausschliesslich in der HAL abgezogen (drone_hal.cpp, 3-s-Startup-Mittelung);
%   in der Sim bildet sensors.slx diese Stufe nach (Abzug von imu.gyro_bias_hat
%   hinter dem Gyro-Block). Der fruehere Eingang b_ground ist entfallen, weil er
%   auf der Hardware ein zweites Mal abgezogen hat, noch dazu mit einem fiktiven
%   Wert. Bitte nicht wieder einfuehren: die Firmware traegt keine Bias-Logik.
%
%   Ein-/Ausgaenge:
%     imu_gyro [rad/s] (3x1)  : gemessene Koerperdrehrate, bias-korrigiert (HAL)
%     imu_acc  [m/s^2] (3x1)  : gemessene spezifische Kraft (Koerper)
%     q_ext    (4x1)          : externe Mocap-Lage
%     ka, kE   (Skalar)       : Tilt-, Externreferenz-Gain
%     Ts       [s]            : Abtastperiode
%     q_init   (4x1)          : Startlage
%     q_hat    (4x1)          : geschaetzte Lage
%     Omega_hat(3x1)          : Drehrate fuer den Lageregler

persistent q
if isempty(q)
    q = q_init; % Startlage
end

% --- 1) Innovation (nur Richtung, Betrag traegt keine Lageinfo) ---
na = norm(imu_acc);
if na > 1e-6
    v_acc = imu_acc / na; % spezifische Kraft DCM_{be}(a -g*[0;0;-1]) in Körperkoordinaten
    v_hat = quatRotate(q, [0; 0; 1]); % geschaetzte Schwerkraftrichtung in Körperkoordinaten
    e_acc = cross(v_acc, v_hat); % Schätzfehler = Innovationsterm
else
    e_acc = [0; 0; 0]; % Freifall = unbrauchbar
end

% --- 2) Externe Mocap-Lage ---
nE = norm(q_ext);
if nE > 0.5
    qe    = q_ext / nE; % normieren da durch Quantisierung/Rauschen q_ext kein Einheitsquaternion
    q_err = quatMul(quatConj(q), qe); % Koerperinkrement, das \hat{q} -> q_ext dreht
    if q_err(1) < 0 
        q_err = -q_err; % kuerzeste Drehung, da Doppeldeckung des SO(3) durch Quaternionen
    end  
    e_ext = 2 * q_err(2:4); % Rotationsvektor (Kleinwinkel ~ Fehlerwinkel)
else
    e_ext = [0; 0; 0]; % kein gueltiges q_ext -> für Innovationsterm nur Gyro und Accel nutzen
end

% --- 3) Gewichtete Gesamt-Innovation (Koerperframe) ---
omega_mes = ka * e_acc + kE * e_ext;

% --- 5) Korrigierte Drehrate + Quaternion-Propagation ---
omega = imu_gyro + omega_mes; % Eingang der Kinematik (imu_gyro ist HAL-bias-korrigiert)
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
Omega_hat = imu_gyro; % bereits HAL-bias-korrigiert
end





