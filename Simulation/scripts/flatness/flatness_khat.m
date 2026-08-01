function k_hat = flatness_khat(v, q, F, m, g, gamma_khat, tau_lp, Ts)
%#codegen
% flatness_khat  Schub-Skalenschaetzer fuer den flachheitsbasierten Regler.
%   Schaetzt den effektiven Schubfaktor k_hat (reale/kommandierte Schubkraft).
%
%   Messung übrt Mocap, nicht IMU: der Beschleunigungsmesser traegt
%   Vibration + BIAS -> k_hat wuerde auf einen falschen Wert konvergieren. 
%   Hier wird die Beschleunigung einmalig aus der Luenberger-Geschwindigkeit 
%   v_hat differenziert und tiefpassgefiltert. Der Schaetzer ist langsam 
%   (~1-5 Hz) -> laeuft auf GCS-/Mocap-Rate (100 Hz),
%   braucht die 1-kHz-IMU-Rate nicht.
%
%   Eingaenge
%     v        3x1  Geschwindigkeit aus dem Luenberger-Beobachter (mocap-basiert)
%     q        4x1  Lage-Quaternion (fuer die Schubachse zB = R(:,3))
%     F        1x1  kommandierter Schub aus flatness_ctrl [N] (1 Takt verzoegert)
%     m,g      1x1  Masse, Erdbeschleunigung
%     gamma_khat 1x1  Adaptionsgain (init_flatness)
%     tau_lp   1x1  TP-Zeitkonstante auf die differenzierte v [s] (0.10)
%     Ts       1x1  Abtastzeit dieses Schaetzers [s] (== Mocap-/GCS-Takt, 0.01)
%   Ausgang
%     k_hat    1x1  Schub-Skalenschaetzung, geklemmt [0.5, 1.5]

persistent k_hat_s v_prev a_lp init
if isempty(init)
    k_hat_s = 1.0;
    v_prev  = v;
    a_lp    = [0;0;0];
    init    = true;
end

R  = quat2dcm_local(q).';                 % R_{n<-b}
zB = R(:,3);

a_obs = (v - v_prev)/Ts;                  % einmalige Differenzierung der Beobachter-v
alp   = Ts/(Ts + tau_lp);
a_lp  = a_lp + alp*(a_obs - a_lp);        % Tiefpass
s_meas = zB.'*(a_lp + [0;0;g]);           % gelieferter spez. Schub (Projektion auf zB)

% Adaptionsgesetz: s_meas soll dem beabsichtigten spez. Schub (F/m)*k_hat gleichen.
% Gleichgewicht -> k_hat = s_meas / (F/m) = thrust_scale.  (verbatim zur Sim 'mocap_av')
Fspec = F/m;
k_hat_s = k_hat_s + gamma_khat*(s_meas - Fspec*k_hat_s)*Ts;
k_hat_s = min(max(k_hat_s, 0.5), 1.5);

v_prev = v;
k_hat  = k_hat_s;
end
