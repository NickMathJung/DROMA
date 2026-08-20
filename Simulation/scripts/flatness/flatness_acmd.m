function a_cmd = flatness_acmd(F, q, m, g)
%#codegen
%flatness_acmd  Kommandierte Inertialbeschleunigung fuer den Luenberger-Beobachter.
%   a_cmd = (F/m)*zB - [0;0;g],  zB = R(:,3) = Koerper-z in Inertial (aus q).
%
%   Ein-/Ausgaenge:
%     F  : Sollschub-Betrag (skalar)
%     q  : Lage-Quaternion [w x y z] (4x1, scalar-first)
%     m,g: Masse, Erdbeschleunigung
%     a_cmd : kommandierte Inertialbeschleunigung (3x1)

R  = quat2dcm_local(q).'; % R_{n<-b}
zB = R(:,3); % Koerper-z-Achse in Inertial
a_cmd = (F/m)*zB - [0; 0; g];
end
