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
%
% z von 5 auf 7 (05.08.2026): Der Regler bezieht a und jj aus dem INTERNEN
% Zustand zeta1, nicht aus der Messung -- gegen Schubstoerungen ist er auf
% Beschleunigungs-/Ruckebene strukturell blind, nur Position/Geschwindigkeit/
% Integral wehren sich. Die Stoerungsuebertragung ist deshalb
% (s^2+c2*s+c3)/Delta(s) statt s^2/Delta(s), bei 0.46 Hz Faktor ~20 groesser.
% Genau das erklaert die im Flug gemessene z-Schwingung (~11 cm Amplitude bei
% 4.1 % RMS Schubrauschen; das Band 0.3..0.5 Hz ist das Maximum dieser
% Uebertragung, kein externer Oszillator -- der Standlauf war sauber).
% Sim mit gemessener Stoerung + 24 ms Totzeit + Motor-PT1: Pole 5 -> 10.2 cm,
% Pole 7 -> 5.8 cm, robust auch bei verdoppelter Totzeit (48 ms: 6.0 cm,
% keine Aufklingtendenz; Pole 8 waere mit 4.6 cm moeglich, aber ein Schritt
% nach dem anderen). Das z-A/B (Flug 4, 05.08.2026) hat die Rechnung bestaetigt:
% e_z RMS im Halten 7.3 -> 3.5 cm, der 0.46-Hz-Ton ist verschwunden.
%
% x,y von 4 auf 5 (nach dem z-A/B): dieselbe Blindheit gilt quer -- Stoerungen
% am Schubvektor (Kipp-Komponente) sehen a/j ebenfalls nicht. Stiffness-Gewinn
% (s+4)^4 -> (s+5)^4 im relevanten Band ~2x, gleiche Schrittweite (Faktor 1.25)
% wie das validierte z 5 -> 7 (1.4). Rollautoritaet ist 5.8x groesser als Gier,
% Budget unkritisch.
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
% Adaptionsgain (0 = Schaetzer aus, k_hat bleibt auf k_hat0). Effektive
% Zeitkonstante 1/(gamma*F/m) ~ 8.5 s. War 0.05 (~1.7 s) und lag damit nur einen
% Faktor 1.7 vom Positions-Integrierer (Pol 1 rad/s, tau = 1 s) entfernt: zwei
% Integratoren aehnlicher Geschwindigkeit, die DIESELBE konstante Schubabweichung
% ausregeln. Im Flug 05.08.2026 waren k_hat und aint_z im Halten zu +0.875
% korreliert, k_hat wanderte 0.884 -> 0.814 -> 0.876 und aint_z lief auf -189
% (47 % der Klemme). Mit 0.01 trimmt der Schaetzer nur noch die Batteriedrift.
% Belegt im S-4-Replay (ehrliches Streckenmodell): e_z Spitze-Spitze im Halten
% 14.8 -> 12.6 cm, corr(k_hat, aint_z) -0.17 -> 0.00, RMS unveraendert. Ueber
% k_thrust 0.78/0.84/0.92 kostet die Verlangsamung <= 1 % RMS gegenueber 0.05.
% gamma = 0 waere in z noch etwas ruhiger, laedt aber bei k_thrust 0.78 den
% Positions-Integrierer auf -293 von 400 -- als Gegentest gut, dauerhaft zu eng.
fctrl.gamma_khat = 0.01;
fctrl.tau_lp     = 0.10; % TP auf differenzierte Beobachter-v
% Startwert. Aus dem Halteflug 05.08.2026 (FLAT001) bei 1.585 m ueber die
% Impulsbilanz m*(zdd+g)/(F_cmd*cos(Neigung)): k = 0.838 +- 0.035, der Schaetzer
% fand parallel 0.847. Die 0.898 des Vorflugs waren ueber nur 0.8 m gemessen.
% Die Drohne liefert real rund 15 % WENIGER Schub als das Modell annimmt.
% (Die 1.2, die man am Boden misst, sind Bodeneffekt.)
% Im Replay der groesste Einzelgewinn: RMS ab Steigflug 8.17 -> 6.01 cm,
% max 22.6 -> 15.2 cm -- der Steigflug haengt nicht mehr hinterher.
fctrl.k_hat0     = 0.85;

% --- Hoehenrampe fuer Integration und Adaption --------------------------------
% Beide werden nahe am Boden zurueckgenommen: dort verfaelscht der Bodeneffekt die
% Messung, und der Positions-Integrierer laedt sich auf, ohne dass etwas
% auszuregeln waere. Bewusst eine RAMPE und kein Schalter — so ist nicht nur der
% Zustand stetig, sondern auch seine Ableitung, und im Sinkflug faehrt man nicht
% durch eine Schaltschwelle. Die Rampenbreite (0.15 m) stammt aus der
% Hoehenstaffelung des Flugs: unter 0.25 m weicht k_hat um 8-16 % ab, ab 0.40 m
% nur noch um 6 %.
% Das Band liegt seit dem 05.08.2026 15 cm tiefer als urspruenglich (war
% 0.25/0.40): die Landetrajektorie endet 10 cm ueber dem Boden, also UNTERHALB
% der alten Untergrenze. Im Flug FLAT001 flatterte w in der Endschwebe deshalb
% zwischen 0 und 0.85, und der Positions-Integrierer konnte den Restversatz von
% +5..+19 cm nicht wegnehmen. Die Breite bleibt 0.15 m, die Rampensteigung im
% Modell (G_w_ramp = 1/(hi-lo)) aendert sich also nicht.
% Belegt ist das NUR aus dem Flug: die Simulation kennt keinen Bodeneffekt, dort
% ist die Endschwebe mit allen drei Baendern gleich gut (e_z Ende ~ 0, p-p 3 cm).
fctrl.z_adapt_lo = 0.15; % [m ueber z_ground] w = 0
fctrl.z_adapt_hi = 0.30; % [m ueber z_ground] w = 1

% --- Referenz-Vorhalt ---------------------------------------------------------
% gcu_flat wertet die Trajektorie bei t + T_lead aus (Bias hinter der Digital
% Clock). Grund: im Box-Flug 05.08.2026 hinkte die Position der Referenz um
% -12 cm je m/s hinterher -- auf Hin- UND Rueckweg vorzeichenrichtig, also ein
% reiner Zeitverzug von ~120 ms (Funk ~25 ms + Motor-PT1 30 ms + Lageaufbau).
% Das ist ein Phasenfehler der VORSTEUERUNG, kein Reglerproblem: der Vorhalt
% schiebt die Referenz um die Kettenlaufzeit vor, damit die Drohne im
% Raum-Zeitplan liegt. In Rasten (Ableitungen 0) aendert er nichts.
% 100 ms bewusst etwas unter den gemessenen ~120 ms -- Feintuning per A/B.
fctrl.T_lead = 0.10; % [s]

% --- Abtastzeiten -------------------------------------------------------------
fctrl.Ts_fast = 0.001; % innerer Regler (MCU, 1 kHz)
fctrl.Ts_slow = 0.01; % Schaetzer (GCS/Mocap, 100 Hz)

% quadcop nur zur Konsistenzpruefung referenziert (m,g,J kommen als Ports)
fctrl.m = quadcop.m; 
fctrl.g = quadcop.g; 
fctrl.J = quadcop.J;
end
