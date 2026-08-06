function a_cmd = flatness_acmd(F, q, m, g)
%#codegen
%flatness_acmd  Kommandierte Inertialbeschleunigung fuer den Luenberger-Beobachter.
%   a_cmd = (F/m)*zB - [0;0;g],  zB = R(:,3) = Koerper-z in Inertial (aus q).
%   F ist der Sollschub [N] aus flatness_ctrl, m, g Masse/Erdbeschleunigung.
%
%   WICHTIG: Auf F (oder auf a_cmd) muss upstream ein Unit-Delay (1/z) sitzen,
%   sonst entsteht die algebraische Schleife flatness_ctrl -> F -> a_cmd ->
%   Beobachter -> v -> flatness_ctrl. Ein Schritt Verzoegerung ist vernachlaessigbar.
%
%   Ein-/Ausgaenge:
%     F  : Sollschub-Betrag [N] (skalar)
%     q  : Lage-Quaternion [w x y z] (4x1, scalar-first)
%     m,g: Masse [kg], Erdbeschl. [m/s^2]
%     a_cmd : kommandierte Inertialbeschleunigung [m/s^2] (3x1) -> Beobachter-Eingang

R  = quat2dcm_local(q).'; % R_{n<-b} (Koerper -> Inertial)
zB = R(:,3); % Koerper-z-Achse in Inertial
a_cmd = (F/m)*zB - [0; 0; g];
end
