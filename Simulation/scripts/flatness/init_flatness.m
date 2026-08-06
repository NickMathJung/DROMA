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
eigenvalPos = 1.0*[ 5.0   5.0   5.0   5.0;     % x
                    5.0   5.0   5.0   5.0;     % y
                    7.0   7.0   7.0   7.0];    % z
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
% Yaw: Doppelintegrator-Pole + Integrator (Pol 1 rad/s). Das Giermoment-
% Gleichgewicht ist nicht null und wandert (Box 05.08.2026: -13..+8 deg, bis
% ~28 % der Gierautoritaet) -> ohne Integrator bleibender Fehler
% tau_d/(J_zz*c3). Kette (s+3)(s+6)(s+1); Anti-Windup-Klemme und w_adapt-Blend
% sitzen in flatness_ctrl (AINT_PHI_MAX).
eigenvalPhi = [3 6];
eigenvalIntPhi = 1.0;
coefPhi = conv(conv([1 eigenvalPhi(2)], [1 eigenvalPhi(1)]), [1 eigenvalIntPhi]);

fctrl.eigenvalPos = eigenvalPos;
fctrl.eigenvalInt = eigenvalInt;
fctrl.eigenvalPhi = eigenvalPhi;
fctrl.eigenvalIntPhi = eigenvalIntPhi;
fctrl.coefPos = coefPos;                 
fctrl.coefPhi = coefPhi;

% --- Schub-Skalenschaetzer (flatness_khat) ------------------------------------
fctrl.gamma_khat = 0.01;
fctrl.tau_lp     = 0.10; % TP auf differenzierte Beobachter-v
fctrl.k_hat0     = 0.85;

% --- Hoehenrampe fuer Integration und Adaption --------------------------------
fctrl.z_adapt_lo = 0.15; % [m ueber z_ground] w = 0
fctrl.z_adapt_hi = 0.30; % [m ueber z_ground] w = 1

% --- Referenz-Vorhalt ---------------------------------------------------------
% gcu_flat wertet die Trajektorie bei t + T_lead aus da Totzeit im System
fctrl.T_lead = 0.10; % [s]

% --- Abtastzeiten -------------------------------------------------------------
fctrl.Ts_fast = 0.001; % innerer Regler (MCU, 1 kHz)
fctrl.Ts_slow = 0.01; % Schaetzer (GCS/Mocap, 100 Hz)

% quadcop nur zur Konsistenzpruefung referenziert (m,g,J kommen als Ports)
fctrl.m = quadcop.m; 
fctrl.g = quadcop.g; 
fctrl.J = quadcop.J;
end
