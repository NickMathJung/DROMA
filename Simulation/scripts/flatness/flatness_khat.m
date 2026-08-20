function k_hat = flatness_khat(v, q, F, m, g, gamma_khat, tau_lp, Ts, k_hat0, w_adapt)
%#codegen
% flatness_khat  Schub-Skalenschaetzer fuer den flachheitsbasierten Regler.
%   Schaetzt den effektiven Schubfaktor k_hat (reale/kommandierte Schubkraft).
%
%   Die Beschleunigung wird aus der Luenberger-Geschwindigkeit v_hat einmalig
%   differenziert und tiefpassgefiltert; der Schaetzer laeuft auf Mocap-Rate.
%
%   Eingaenge
%     v        3x1  Geschwindigkeit aus dem Luenberger-Beobachter
%     q        4x1  Lage-Quaternion
%     F        1x1  kommandierter Schub
%     m,g      1x1  Masse, Erdbeschleunigung
%     gamma_khat 1x1  Adaptionsgain
%     tau_lp   1x1  TP-Zeitkonstante auf die differenzierte v
%     Ts       1x1  Abtastzeit dieses Schaetzers
%     k_hat0   1x1  Startwert
%     w_adapt  1x1  Adaptions-Freigabe in [0,1]
%   Ausgang
%     k_hat    1x1  Schub-Skalenschaetzung, geklemmt [0.5, 1.5]

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

% Adaptionsgesetz, Gleichgewicht: k_hat = s_meas / (F/m)
Fspec = F/m;
k_hat_s = k_hat_s + w_adapt*gamma_khat*(s_meas - Fspec*k_hat_s)*Ts;
k_hat_s = min(max(k_hat_s, 0.5), 1.5);

v_prev = v;
k_hat = k_hat_s;
end
