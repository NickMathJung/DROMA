function [imu, mocap] = init_sensors(quadcop, Ts_inner, Ts_mocap)
%init_sensors initializes the imu and the mocap with parameters from their
%   datasheets
arguments (Input)
    quadcop struct % struct holding quadrocopter parameters
    Ts_inner (1,1) double % base sample time of simulation
    Ts_mocap (1,1) double % sample time motion capture system (simulation)
end

arguments (Output)
    imu struct % holding imu parameters from datasheet for simulation
    mocap struct % holding mocap parameters for simulation
end

imu.gyro_FSR = deg2rad(500); % rad/s  (FS_SEL=1: +-500 deg/s, 65.5 LSB/(deg/s))
imu.Ts = Ts_inner; % Update-Periode des IMU-Blocks (Sekunden)
imu.location = [-0.014; -0.015; 0.045];
 
% --- Gyroskop ---
% Roher Sensor-Bias, wie er im MPU-Register steht (vor Kalibrierung). Die
% Zero-Initialtoleranz liegt bei +-20 deg/s; dieser Wert ist repraesentativ:
imu.gyro_bias  = deg2rad([10; -10; 10]);   % rad/s   (Spec-Grenze +-0.349 rad/s)

% Was die HAL schaetzt: drone_hal.cpp mittelt den Bias im 3-s-Startup und zieht
% ihn vom Rohwert ab, bevor imu_gyro die MCU erreicht.
%
% Die Reihenfolge ist sicherheitsrelevant, deshalb fest verdrahtet:
%   sensors.slx ('Three-axis Gyroscope') praegt gyro_bias auf und saettigt erst
%   danach bei +-gyro_FSR. Der Bias liegt also vor der Saturation, wie auf echter
%   Hardware. Erst dahinter zieht der HAL-Nachbau (Sum 'HAL gyro bias' -> Bus
%   Creator) gyro_bias_hat wieder ab. mcu.slx zieht nichts mehr ab (kein
%   Constant1/Subtract, kein Mahony-b_ground) — sonst wuerde der Bias doppelt
%   subtrahiert, genau der Bug, der auf HW 10 deg/s Schein-Drehrate erzeugt hat.
%
% gyro_bias=0 waere keine Vereinfachung, sondern zu optimistisch: dann saehe die
% Saturation den Bias nicht. Die Marge ist hier knapp (FSR 8.727 gegen
% safety.omega_max 8.5 rad/s = 0.227 Marge), davon frisst |bias|=0.175 rund 77 %.
%
% gyro_bias_hat == gyro_bias bedeutet perfekte Kalibrierung (Default). Fuer den
% Kalibrier-Restfehler hier abweichen lassen, z.B.
%   imu.gyro_bias_hat = imu.gyro_bias + deg2rad([0.05; -0.03; 0.04]);
imu.gyro_bias_hat = imu.gyro_bias;         % rad/s   (HAL-Schaetzung)
imu.gyro_ASD   = deg2rad(0.005);           % rad/s/sqrt(Hz)  (Amplituden-Spektraldichte)
imu.gyro_PSD   = imu.gyro_ASD^2;           % (rad/s)^2/Hz    -> Band-Limited White Noise "Noise power"
% Skalenfaktor-Toleranz +-3 %, Kreuzachsen-Empfindlichkeit +-2 % -> 3x3-Matrix
imu.gyro_M     = [ 1.03  0.02  0.02;
                  -0.02  0.97  0.02;
                   0.02 -0.02  1.03];
% G-empfindlicher Bias (Linear Acceleration Sensitivity 0.1 deg/s/g)
imu.gyro_gsens = deg2rad(0.1)*[1;1;1];     % rad/s pro g
% Bandbreite (DLPF, hier 100 Hz) als 2nd-order dynamics
imu.gyro_wn    = 2*pi*30;                 % rad/s
imu.gyro_zeta  = 0.707;
 
% --- Accelerometer ---
imu.acc_FSR = 4*quadcop.g; % m/s^2  (AFS_SEL=1: +-4 g, 8192 LSB/g)
% Messbias (Zero-G: X/Y +-50 mg, Z +-80 mg):
imu.acc_bias   = 1.0*[0.05; -0.05; 0.08]*quadcop.g; % m/s^2
% Rauschen: Power Spectral Density 400 ug/sqrt(Hz)
imu.acc_ASD    = 400e-6*quadcop.g; % (m/s^2)/sqrt(Hz)
imu.acc_PSD    = imu.acc_ASD^2; % (m/s^2)^2/Hz -> Band-Limited White Noise "Noise power"
% Skalenfaktor-Toleranz +-3 %, Kreuzachsen +-2 %, Nichtlinearitaet 0.5 %
imu.acc_M      = [ 1.03  0.02  0.02;
                  -0.02  0.97  0.02;
                   0.02 -0.02  1.03];
% Bandbreite (DLPF, hier 100 Hz)
imu.acc_wn     = 2*pi*8; % rad/s
imu.acc_zeta   = 0.707;

% Motive @ Ts_mocap   (TODO: aus OptiTrack-Spezifikation/Messung)
mocap.pos_noise = 1e-3; % RMS
mocap.att_noise = 0.5*pi/180; % RMS
mocap.Ts_mocap  = Ts_mocap; % Sample-Periode (= 1/f_base)
mocap.t_delay   = 0.008; % optional Transportverzoegerung
mocap.dropout_p = 0.01; % Wahrscheinlichkeit ausfall pro Sample

% --- Reales Mocap (OptiTrack/Motive via NatNet) -----------------------------
% Nur fuer den Pruefstand bench.slx (Block "Motive" = MotiveMocap). Die reine
% Simulation (quadcop.slx) nutzt das nicht, dort modelliert sensors.slx die
% Mocap mit den Rauschwerten oben.
% Laufen Motive und MATLAB auf einem Rechner, sind beide IPs 127.0.0.1. Sonst
% ist host_ip der Motive-Rechner und client_ip dieser Rechner.
mocap.host_ip      = '127.0.0.1';
mocap.client_ip    = '127.0.0.1';
% Streaming-ID des Drohnen-Rigid-Body in Motive (Assets-Pane); muss zur
% geflogenen Drohne passen.
mocap.streaming_id = 1;
% Motive muss auf Z-Up streamen (Settings -> Streaming -> Up Axis = Z), passend
% zum z-up-Projekt. MotiveMocap transformiert absichtlich nicht: eine zweite
% Korrekturstelle waere wieder eine doppelte Kompensation. NatNet liefert Meter
% und Quaternionen scalar-last; die Umsortierung auf scalar-first [w x y z]
% passiert einmalig in MotiveMocap.
end