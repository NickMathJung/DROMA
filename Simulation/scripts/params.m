%% params.m  --  Zentrale Parameterdatei
%  Aufruf ueber die PreLoadFcn des Top-Modells:  run(fullfile(projRoot,'scripts','params.m'))
clear;
clc;
close all;
%% ------------------------------------------------------------------ Raten
% Base rate = Ts_inner. Every other sample time in the model has to be an
% integer multiple of Ts_inner, otherwise the ode4 fixed-step scheduler is invalid.
f_base = 100; % Grundrate
rate_outer2inner = 10; % factor by how much the controller on the drone is sampled faster compared to the ground station
Ts_inner = 1/(rate_outer2inner*f_base);            
Ts_sim = Ts_inner; % Fixed-step Grundschrittweite (ode4)

% Multiples of Ts_inner (= rate_outer2inner*Ts_inner = 1/f_base each)
Ts_mocap = rate_outer2inner*Ts_inner; % Optitrack
Ts_gcs = rate_outer2inner*Ts_inner; % Beobachter + Positionsregler
Ts_link = rate_outer2inner*Ts_inner; % Funkstrecke
Ts_batt = 100*Ts_gcs; % Rate der Batterieüberwachungsfunktion

%% -------------------------------------------------------------- Modellparameter
quadcop = init_quadcop();

%% ------------------------------------------ IMU: MPU-6050 (Datenblatt)
% Vollausschlag (FSR) -> externe Saettigung; Auswahl konfigurierbar
[imu, mocap] = init_sensors(quadcop, Ts_inner, Ts_mocap);

%% --------------------------------------------------------- Funkstrecke
link_params = init_link(quadcop, Ts_inner);

%% ------------------------------------------------------------- Regler
controller = init_controller(quadcop);

%% ------------------------------------------------------------ Safety
safety = init_safety(quadcop);

%% ------------------------------------------------------------ Schaetzer
[mahony,luen] = init_estimator(Ts_gcs);

%% ------------------------------------------------------------ Trajektorie
traj = init_trajectory();

%% ------------------------------------------------------------ Batterie management
safety = init_battery_manag(quadcop, safety, Ts_batt);

%% ------------------------------------------------------------ Supervisor (Soft-Land)
supervisor = init_supervisor(quadcop,Ts_gcs);

