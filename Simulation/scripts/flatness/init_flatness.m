function fctrl = init_flatness(quadcop)
%init_flatness  Auslegung des flachheitsbasierten Folgereglers + Schub-Schaetzers.
%   Analog zu init_controller (Kaskade). Liefert die Brunovsky-Koeffizienten aus
%   der Polvorgabe der flachen Fehlerdynamik sowie die Schaetzer-/Takt-Parameter.
arguments (Input)
    quadcop struct
end
arguments (Output)
    fctrl struct
end

% --- Brunovsky-Polvorgabe der flachen Fehlerdynamik ---------------------------
% Je Achse ein Satz reeller Pole [rad/s] fuer die Integratorkette (Pos bis Snap).
% z (Zeile 3) schneller gewaehlt (wie in der Kaskade omega_pos_z > x,y).
eigenvalPos = [ 8.0   8.0   8.0   8.0;     % x
                8.0   8.0   8.0   8.0;      % y
                10.0  10.0  10.0  10.0 ];   % z
coefPos = zeros(3,5);
for jr = 1:3
    cp = [1 eigenvalPos(jr,1)];
    for ii = 1:3
        cp = conv(cp, [1 eigenvalPos(jr,ii+1)]);
    end
    coefPos(jr,:) = cp;
end
eigenvalPhi = [1 2];                     % Yaw: Doppelintegrator-Pole
coefPhi = conv([1 eigenvalPhi(2)], [1 eigenvalPhi(1)]);

fctrl.eigenvalPos = eigenvalPos;
fctrl.eigenvalPhi = eigenvalPhi;
fctrl.coefPos = coefPos;                 % 3x5  -> flatness_ctrl
fctrl.coefPhi = coefPhi;                 % 1x3  -> flatness_ctrl

% --- Schub-Skalenschaetzer (flatness_khat) ------------------------------------
fctrl.gamma_khat = 0.05;                 % Adaptionsgain: langsam (tau~2s) -> faengt
                                         % quasistat. Download, ohne Manoever-z-Ripple
                                         % anzufachen. Sim: z-max 3.9->1.6cm, RMS ->4.3mm.
fctrl.tau_lp     = 0.10;                 % TP auf differenzierte Beobachter-v [s]
fctrl.k_hat0     = 1.0;                  % Startwert

% --- Abtastzeiten -------------------------------------------------------------
fctrl.Ts_fast = 0.001;                   % innerer Regler (MCU, 1 kHz)
fctrl.Ts_slow = 0.01;                    % Schaetzer (GCS/Mocap, 100 Hz)

% quadcop nur zur Konsistenzpruefung referenziert (m,g,J kommen als Ports)
fctrl.m = quadcop.m; fctrl.g = quadcop.g; fctrl.J = quadcop.J;
end
