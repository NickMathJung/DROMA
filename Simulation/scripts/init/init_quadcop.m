function quadcop = init_quadcop()
%init_quadcop initializes the parameters for the quadcopter

arguments (Output)
    quadcop struct
end

quadcop.g   = 9.81; % m/s^2
quadcop.m   = 0.985; % kg            
quadcop.J   = diag([6.583 * 10^-3, 5.125 * 10^-3, 1.104 * 10^-2]); % kg*m^2 
quadcop.J_inv = inv(quadcop.J);
l = 0.124;  % Armlaenge 

% c_T = 1.134e-3; % (MA-Finn)  
% c_T = 7.433e-5; % N/(s)^2 (selbst aus Datenblatt ohne Faktor 2pi und mit anderem Fit (polyfit)
% --- Schubkonstante aus Datenblatt GEMFAN 51499 ---
% Aus dem Datenblatt (inklusive 0-Punkt)
rpm = [0, 4957, 9256, 12419, 15418, 18497, 20977, 23272, 25355, 27260, 28597];
thrust_g = [0, 48.2, 168.1, 314.5, 496.0, 722.2, 932.4, 1141.5, 1357.9, 1590.0, 1690.8];
% Umrechnung in SI-Einheiten (Winkelgeschwindigkeit in rad/s, Schub in Newton)
omega = rpm .* (2*pi/60);
T = thrust_g .* (9.81/1000);
% Least-Squares-Schätzung für das Modell T = c_T * omega^2
omega_sq = (omega.^2)'; 
T_col = T';
% Berechnung der Schubkonstante
c_T = omega_sq \ T_col;
% Zur Kontrolle: Die berechnete Konstante anzeigen
disp(['Schubkonstante c_T: ', num2str(c_T), ' Ns^2/rad^2']);

% c_tau = 7.398e-6; % (MA-Finn)
% --- Gierkonstante aus Datenblatt GEMFAN 51499 ---
U = 22.2;
I = [0.44, 1.68, 3.55, 6.36, 10.50, 14.95, 20.01, 27.35, 35.50, 42.20];
rpm_Q = [4957, 9256, 12419, 15418, 18497, 20977, 23272, 25355, 27260, 28597];
omega_Q = rpm_Q .* (2*pi/60);
eta = 0.75; % angenommener Motorwirkungsgrad aus Literatur
% P_mech = c_Q * omega^3  =>  c_Q = (omega^3) \ P_mech
P_mech = (eta .* U .* I)';
omega_cube = (omega_Q.^3)';
c_tau = omega_cube \ P_mech;
disp(['Gierkonstante c_tau: ', num2str(c_tau), ' Nms^2/rad^2']);

% --- Umrechnung von Omega zu Throttle (spannungsnormiert auf 22.2 V) ----------
%
% AUS EIGENER MESSUNG, nicht aus dem Datenblatt. Das Datenblatt taugt fuer c_T
% (reine Aerodynamik, Schub/rpm^2 ist dort ueber den ganzen Bereich auf +-4 %
% konstant), aber NICHT fuer die Abbildung Throttle -> Drehzahl:
%   Es nennt bei 22.2 V/10 % schon 4957 rpm, waehrend kv*U*duty = 1900*22.2*0.1
%   = 4218 rpm die LEERLAUFdrehzahl waere. Ein belasteter Motor kann die nie
%   erreichen. Auf das Datenblatt gestuetzt lag das Modell durchweg 22 % zu hoch
%   in der Drehzahl, also Faktor 0.67 im Schub -- beim rechnerischen Hover-
%   Throttle waeren real nur 69 % des Gewichts herausgekommen.
%
% Messung: Einzelmotor am Pruefstand, Propeller montiert, Blattdurchgangs-
% frequenz aus dem Audiospektrum (Handyaufnahme, Welch-PSD, Peak ueber lokalem
% Median). Dreiblattschraube -> rpm = f_Blatt/3*60. Zwei Reihen bei leicht
% verschiedener Akkuspannung, deshalb U je Punkt mitgefuehrt.
thr_meas = [ 10,    15,    20,    25,    30,    35,    40,    45   ];  % [%] Kommando
f_blade  = [147.4, 213.3, 276.6, 339.1, 399.0, 457.6, 504.0, 559.9];  % [Hz]
U_meas   = [ 15.90, 15.90, 15.90, 15.90, 15.90, 15.66, 15.66, 15.66]; % [V]
n_blades = 3;

omega_meas = (f_blade/n_blades) * 2*pi;              % [rad/s]
% Spannungsnormierung: omega haengt vom Produkt duty*U ab, fuer dieselbe Drehzahl
% gilt throttle(U) = throttle(22.2 V)*22.2/U. Das Polynom liefert also den
% AEQUIVALENTEN 22.2-V-Throttle; die Division durch die gemessene Spannung macht
% das Modell zur Laufzeit (V_filt aus safety_battery, geklemmt).
throttle_eq = thr_meas .* U_meas / 22.2;

% Fit ohne Konstante: throttle = p1*omega^2 + p2*omega. Durch den Ursprung, weil
% omega = 0 exakt throttle = 0 bedeutet. Normiert gerechnet (omega/1e3), sonst
% ist die Normalengleichung schlecht konditioniert.
Mfit = [(omega_meas(:)/1e3).^2, omega_meas(:)/1e3];
cfit = Mfit \ throttle_eq(:);
quadcop.p_from_omega = [cfit(1)/1e6, cfit(2)/1e3, 0];      % polyval-Reihenfolge
disp('Koeffizienten fuer die Abbildung (omega -> aequiv. 22.2-V-Throttle):');
disp(quadcop.p_from_omega);
% Guete: max. Residuum 0.37 Prozentpunkte (die meisten unter 0.1). Der Hover
% liegt bei 1133 rad/s und damit INNERHALB des Messbereichs (45 % -> 1173 rad/s),
% es wird also interpoliert, nicht extrapoliert.
% Offen: alle Punkte stammen von ~15.8 V. Das Spannungsgesetz 22.2/U ist damit
% nicht ueber verschiedene Spannungen geprueft.

% Spannungsnormierung: thr = polyval(p_from_omega, omega) * U_ds / clamp(V_filt).
% Die Klemmung ist NICHT optional — V_filt steht im Nenner, und ein ausgefallener
% Batteriesensor (V_filt = 0) wuerde sonst alle vier Motoren auf 100 % treiben.
quadcop.U_ds = 22.2; % [V] Spannung, bei der das Datenblatt aufgenommen wurde
quadcop.V_thr_min = 11.0; % [V] untere Klemme (4S leer ~12.0, etwas Luft darunter)
quadcop.V_thr_max = 17.5; % [V] obere Klemme (4S voll 16.8, etwas Luft darueber)
quadcop.V_thr_init = 16.8;  % [V] 4S voll geladen

% Motorwinkelgeschwindigkeiten -> Schubkraft und Drehmomente:  [F; tau_x; tau_y; tau_z] = Gamma * [w1^2; w2^2; w3^2; w4^2] 
% momentan fliegt Quadrkopter in X-Konfiguration
% konfiguriere Gamma:
alpha = 38.4; % degrees (Winkel aus Lunze/Schwung Paper)
% Quadrcopterkonfiguration ist ein Mix aus H- und X-Konfiguration 
a = l * sin(alpha / 180 * pi);
b = l * cos(alpha / 180 * pi);
% Position der Rotoren im Körperkoordinatensystem
r1 = [ a; -b;  0];
r2 = [ a;  b;  0];
r3 = [-a;  b;  0];
r4 = [-a; -b ; 0];

F = [c_T  c_T  c_T  c_T];
f_i = [0; 0; c_T];
tau_i = [0; 0; c_tau];
tau = [skew(r1)*f_i-tau_i  skew(r2)*f_i+tau_i  skew(r3)*f_i-tau_i  skew(r4)*f_i+tau_i];
quadcop.Gamma = [F; tau];

quadcop.Gamma_inv = inv(quadcop.Gamma);

% uncomment to test robustness
% test_c_tau = 2; % factor
% test_c_T = 1.2; % factor
% quadcop.Gamma = [test_c_tau*quadcop.Gamma(1:3,:); test_c_tau*quadcop.Gamma(end,:)]; 


% Motor (PT1) + Saettigung
quadcop.tau_m = 0.030; % Motorzeitkonstante 
quadcop.rotors_min = 0; % rad/s
quadcop.rotors_max = 30000; % rad/s  

quadcop.T = [1 0 0; 0 -1 0; 0 0 -1]; % to transform form NED to laboratory coordinate system (all with z-up)

quadcop.x0 = [0;0;0];
quadcop.v0 = [0;0;0];
quadcop.q0 = [1;0;0;0];
quadcop.euler0 = [0;0;0];
quadcop.w0 = [0;0;0];


end

% --- Helfer ---

function schiefsym = skew(r)
% skew gibt die schiefsymmetrische Matrix schiefsym zu r zurück

arguments (Input)
    r (3,1) double
end

arguments (Output)
    schiefsym (3,3) double
end

schiefsym = [ 0    -r(3)  r(2);
              r(3)  0    -r(1);
             -r(2)  r(1)  0];
end