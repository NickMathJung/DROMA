%% params.m  --  Zentrale Parameterdatei
%  wird über die PreLoadFcn von DROMA.prj aufgerufen

clear;
clc;
close all;

%% ------------------------------------------------------------------ Raten
% Grundtaktrate = Ts_inner: Jede andere Sampletime muss ein ganzzahliges 
% vielfaches von Ts_inner sein, sonst streikt der fixed-step scheduler
f_base = 100; % Grundtaktrate
rate_outer2inner = 10; % Faktor um den MCU auf Drohne schneller sein soll als Takt in SIMULINK
Ts_inner = 1/(rate_outer2inner*f_base);            
Ts_sim = Ts_inner; % Fixed-step Grundschrittweite 

% Vielfache von Ts_inner (= rate_outer2inner*Ts_inner = 1/f_base)
Ts_mocap = rate_outer2inner*Ts_inner; % Optitrack
Ts_gcs = rate_outer2inner*Ts_inner; % SIMULINK
Ts_link = rate_outer2inner*Ts_inner; % Funkstrecke
Ts_batt = 100 * Ts_gcs; % Rate der Batterieüberwachungsfunktion

%% -------------------------------------------------------------- Modellparameter
quadcop = init_quadcop();

%% ------------------------------------------ IMU: MPU-6050 
[imu, mocap] = init_sensors(quadcop, Ts_inner, Ts_mocap);

%% --------------------------------------------------------- Funkstrecke
link_params = init_link(quadcop, Ts_inner);

%% ------------------------------------------------------------- Regler
controller = init_controller(quadcop);

%% ------------------------------------------------------------ Safety
safety = init_safety(quadcop);

%% ------------------------------------------------------------ Schätzer
[mahony,luen] = init_estimator(Ts_gcs);

%% ------------------------------------------------------------ Trajektorie
traj = init_trajectory();

%% ------------------------------------------------------------ Batterie-Management
safety = init_battery_manag(quadcop, safety, Ts_batt);

%% ---------------------------------------------- Flatness-Variante 
fctrl = init_flatness(quadcop);

%% ------------------------------------------------------------ Supervisor 
supervisor = init_supervisor(quadcop,Ts_gcs);

