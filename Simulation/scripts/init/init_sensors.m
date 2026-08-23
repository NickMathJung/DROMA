function [imu, mocap] = init_sensors(quadcop, Ts_inner, Ts_mocap)
%init_sensors initializes the IMU (inertial emeasurement unit) and the mocap 
%   with parameters from their datasheets
arguments (Input)
    quadcop struct % struct holding quadrocopter parameters
    Ts_inner (1,1) double % base sample time of simulation
    Ts_mocap (1,1) double % sample time motion capture system (for simulation)
end

arguments (Output)
    imu struct % holding imu parameters from datasheet for simulation
    mocap struct % holding mocap parameters for simulation
end

imu.gyro_FSR = deg2rad(500); % rad/s
imu.Ts = Ts_inner; % Update-Periode des IMU-Blocks 
imu.location = [-0.014; -0.015; 0.065]; % Offset vom Schwerpunt
 
% --- Gyroskop ---
% Sensor-Bias
imu.gyro_bias  = deg2rad([10; -10; 10]);

% gyro_bias_hat == gyro_bias: perfekte Kalibrierung
imu.gyro_bias_hat = imu.gyro_bias; % rad/s
imu.gyro_ASD   = deg2rad(0.005); % rad/s/sqrt(Hz) (Amplituden-Spektraldichte)
imu.gyro_PSD   = imu.gyro_ASD^2; % (rad/s)^2/Hz

% Kreuzachsen-Empfindlichkeit
imu.gyro_M     = [ 1.03  0.02  0.02;
                  -0.02  0.97  0.02;
                   0.02 -0.02  1.03];

% g-empfindlicher Bias 
imu.gyro_gsens = deg2rad(0.1)*[1;1;1]; % rad/s pro g

% Bandbreite DLPF als 2nd-order dynamics
imu.gyro_wn    = 2*pi*30; % rad/s
imu.gyro_zeta  = 0.707;
 
% --- Accelerometer ---
imu.acc_FSR = 4*quadcop.g; % m/s^2

% Messbias
imu.acc_bias   = 1.0*[0.05; -0.05; 0.08]*quadcop.g; % m/s^2

% Rauschen: Power Spectral Density 400 ug/sqrt(Hz)
imu.acc_ASD    = 400e-6*quadcop.g; % (m/s^2)/sqrt(Hz)
imu.acc_PSD    = imu.acc_ASD^2; % (m/s^2)^2/Hz

% Kreuzachsenkopplung +-2 %
imu.acc_M      = [ 1.03  0.02  0.02;
                  -0.02  0.97  0.02;
                   0.02 -0.02  1.03];

% Bandbreite DLPF
imu.acc_wn     = 2*pi*8; % rad/s
imu.acc_zeta   = 0.707;

% Motive @ Ts_mocap   
mocap.pos_noise = 1e-3; % RMS
mocap.att_noise = 0.5*pi/180; % RMS
mocap.Ts_mocap  = Ts_mocap; % Sample-Periode
mocap.t_delay   = 0.008; % optional Transportverzoegerung
mocap.dropout_p = 0.01; % Wahrscheinlichkeit ausfall pro Sample

% --- Reales Mocap: OptiTrack/Motive via NatNet ---------------------------
% Laufen Motive und MATLAB auf einem Rechner, sind beide IPs 127.0.0.1.
mocap.host_ip      = '127.0.0.1';
mocap.client_ip    = '127.0.0.1';

% Streaming-IDs der Drohnen-Rigid-Bodies in Motive (Assets-Pane).
% Reihenfolge = Drohnenindex.
% Doppelte id (z.B. [1 1]) = Solobetrieb: beide GCS-Pfade regeln dieselbe Drohne.
% mocap.streaming_ids = [4 4 4 4];
mocap.streaming_ids = [1 2 3 4];
% Motive muss auf Z-Up streamen (Settings -> Streaming -> Up Axis = Z).
% NatNet liefert Meter und Quaternionen scalar-last; die Umsortierung auf
% scalar-first [w x y z] passiert einmalig in MotiveMocap.
end