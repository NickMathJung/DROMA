% Read data from SD-Card to evaluate flight data after the flight such as
% IMU data or signals from the flatness based controller

addpath('C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\scripts\flatness');
L1 = read_flight_log_flat('E:\FLAT001.BIN');
save('C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\data\s4_log.mat','L1');
