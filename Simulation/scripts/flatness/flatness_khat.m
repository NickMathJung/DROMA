function k_hat = flatness_khat(v, q, F, m, g, gamma_khat, tau_lp, Ts, k_hat0, w_adapt)
%#codegen
% flatness_khat  Schub-Skalenschaetzer fuer den flachheitsbasierten Regler.
%   Schaetzt den effektiven Schubfaktor k_hat (reale/kommandierte Schubkraft).
%
%   Messung über Mocap, nicht IMU: der Beschleunigungsmesser traegt
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
%     tau_lp   1x1  TP-Zeitkonstante auf die differenzierte v (0.10)
%     Ts       1x1  Abtastzeit dieses Schaetzers [s] (== Mocap-/GCS-Takt, 0.01)
%     k_hat0   1x1  Startwert (init_flatness). War frueher fest 1.0 verdrahtet —
%                   fctrl.k_hat0 existierte, kam hier aber nie an.
%     w_adapt  1x1  Adaptions-Freigabe in [0,1], Hoehenrampe aus dem Modell.
%   Ausgang
%     k_hat    1x1  Schub-Skalenschaetzung, geklemmt [0.5, 1.5]
%
%   Warum die Freigabe: nahe am Boden hebt der Bodeneffekt den realen Schub an,
%   und s_meas wird zusaetzlich verfaelscht, sobald die Drohne aufsetzt (dann
%   traegt der Boden mit und a_lp sagt nichts mehr ueber den Schub). Gemessen im
%   Flug 05.08.2026: k_hat = 0.898 +- 0.033 ab 0.8 m Hoehe, aber bis 1.204 am
%   Boden. Dieser falsche Wert bleibt stehen und ginge in den naechsten Start
%   mit — die Drohne haette dort 20 % zu wenig Schub. w_adapt multipliziert nur
%   den Zuwachs, k_hat_s selbst bleibt also stetig.

persistent k_hat_s v_prev a_lp init
if isempty(init)
    k_hat_s = k_hat0;
    v_prev  = v;
    a_lp    = [0;0;0];
    init    = true;
end

R  = quat2dcm_local(q).'; % R_{n<-b}
zB = R(:,3);

a_obs = (v - v_prev)/Ts; % einmalige Differenzierung von v_hat
alp = Ts/(Ts + tau_lp);
a_lp = a_lp + alp*(a_obs - a_lp); % Tiefpass
s_meas = zB.'*(a_lp + [0;0;g]); % spez. Schub (Projektion auf zB)

% Adaptionsgesetz: s_meas soll dem beabsichtigten spez. Schub (F/m)*k_hat gleichen.
% Gleichgewicht -> k_hat = s_meas / (F/m) = thrust_scale
Fspec = F/m;
% Tiefpass und v_prev laufen IMMER mit — nur die Adaption wird zurueckgenommen.
% So ist der Filter eingeschwungen, sobald w_adapt wieder steigt.
k_hat_s = k_hat_s + w_adapt*gamma_khat*(s_meas - Fspec*k_hat_s)*Ts;
k_hat_s = min(max(k_hat_s, 0.5), 1.5);

v_prev = v;
k_hat = k_hat_s;
end
