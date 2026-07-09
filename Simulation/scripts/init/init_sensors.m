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
% Messbias (ZERO-Initialtoleranz +-20 deg/s, vor Kalibrierung); repraesentativ:
imu.gyro_bias  = deg2rad([10; -10; 10]);   % rad/s   (Spec-Grenze +-0.349 rad/s)
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
end