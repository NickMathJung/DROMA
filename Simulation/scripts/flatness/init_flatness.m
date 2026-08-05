function fctrl = init_flatness(quadcop)
%init_flatness  Auslegung des flachheitsbasierten Folgereglers + Schub-Schaetzers.
%   Analog zu init_controller (Kaskade). Liefert die Brunovsky-Koeffizienten aus
%   der Polvorgabe der flachen Fehlerdynamik sowie die Schaetzerparameter.
arguments (Input)
    quadcop struct
end
arguments (Output)
    fctrl struct
end

% --- Brunovsky-Polvorgabe der flachen Fehlerdynamik ---------------------------
% Je Achse ein Satz reeller Pole [rad/s] fuer die Integratorkette (Pos bis Snap).
% z (Zeile 3) schneller gewaehlt (wie in der Kaskade omega_pos_z > x,y).
eigenvalPos = 1.0*[ 4.0   4.0   4.0   4.0;     % x
                    4.0   4.0   4.0   4.0;     % y
                    5.0   5.0   5.0   5.0];    % z
eigenvalInt = 1*[1.0; 1.0; 1.0];
coefPos = zeros(3,6);
for jr = 1:3
    cp = [1 eigenvalPos(jr,1)];
    for ii = 1:3
        cp = conv(cp, [1 eigenvalPos(jr,ii+1)]);
    end
    cp = conv(cp, [1 eigenvalInt(jr)]);   
    coefPos(jr,:) = cp;
end
% Yaw: Doppelintegrator-Pole. Der Kanal hat KEINEN Integrator, ein konstantes
% Stoermoment hinterlaesst also den bleibenden Fehler e = tau_d/(J_zz*c3) --
% c3 = prod(eigenvalPhi) ist der einzige Hebel darauf. [1 2] war zu weich
% (Flug 03.08.2026: bis 28 deg Ausschlag beim Aufsetzen); [3 6] verneunfacht
% die Steifigkeit. Obergrenze ist die Gierautoritaet: sie ist 5.8x schwaecher
% als die Rollautoritaet (tau_z,max = 0.163 Nm bei Schwebedrehzahl), und ein
% 60-deg-Sprung treibt die Drossel damit auf 77% (bei [4 8] schon auf 97%).
% Rauschverstaerkung ist unkritisch (c2*sigma_gyro = 0.06% des Budgets).
eigenvalPhi = [3 6];
coefPhi = conv([1 eigenvalPhi(2)], [1 eigenvalPhi(1)]);

fctrl.eigenvalPos = eigenvalPos;
fctrl.eigenvalInt = eigenvalInt;
fctrl.eigenvalPhi = eigenvalPhi;
fctrl.coefPos = coefPos;                 
fctrl.coefPhi = coefPhi;

% --- Schub-Skalenschaetzer (flatness_khat) ------------------------------------
fctrl.gamma_khat = 0.05; % Adaptionsgain
fctrl.tau_lp     = 0.10; % TP auf differenzierte Beobachter-v 
fctrl.k_hat0     = 1.0; % Startwert

% --- Abtastzeiten -------------------------------------------------------------
fctrl.Ts_fast = 0.001; % innerer Regler (MCU, 1 kHz)
fctrl.Ts_slow = 0.01; % Schaetzer (GCS/Mocap, 100 Hz)

% quadcop nur zur Konsistenzpruefung referenziert (m,g,J kommen als Ports)
fctrl.m = quadcop.m; 
fctrl.g = quadcop.g; 
fctrl.J = quadcop.J;
end
