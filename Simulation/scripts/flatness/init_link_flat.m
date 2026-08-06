function link_flat_params = init_link_flat(quadcop, Ts_inner, Ts_gcs)
%init_link_flat  Link-Parameter der FLATNESS-Variante (Simulation).
%   Gleiches Kanalmodell wie init_link (Latenz + Bernoulli-Verlust + int16-
%   Quantisierung), aber fuer den Bus_Cmd_flat-Inhalt:
%     int16-Teil (21x1): [mocap_pos(3); p_ref(3); v_ref(3); a_ref(3);
%                         j_ref(3); s_ref(3); yaw_ref(3)]
%     quat-Teil  (1x1):  q_ext (smallest-three, uint32)
%     flags      (2x1):  [estop; ack]
%   HINWEIS OTA: 21x int16 + uint32 + flags = ~48 B Nutzlast > 32-B-nRF-Frame.
%   Die Fluggeraet-Firmware wird das auf 2+ Frames splitten (Multi-Frame,
%   Nicks Entscheid); dieses Sim-Modell bildet Quantisierung/Latenz/Verlust
%   ab, nicht das Framing.
arguments (Input)
    quadcop struct
    Ts_inner (1,1) double
    Ts_gcs   (1,1) double = 0.01   % Takt, in dem gcu_flat ein Kommando erzeugt
end
arguments (Output)
    link_flat_params struct
end

% --- Transportlatenz (ganzzahliger Delay of Ts_inner) ---
% Ueber diesen Pfad laufen NICHT nur die Sollwerte, sondern auch mocap_pos und
% q_ext -- also die gesamte Messkette Motive -> GCS -> Funk -> MCU. Die
% frueheren 5 ms bildeten nur den Funk-Hop ab, die Mocap-Pipeline fehlte.
%
% Gemessen am Flug 05.08.2026 (FLAT001): Kreuzkorrelation zwischen
% kommandiertem Schub und der aus Mocap gewonnenen Vertikalbeschleunigung ->
% Maximum bei +50 ms (r = 0.78). Das ist Aktuatorkette PLUS Messkette, denn die
% ausgewertete Mocap-Position ist die, die die MCU gesehen hat. In der
% Simulation ergibt dieselbe Messung +30 ms -- dort wird mocap_pos VOR dem Link
% abgegriffen, es ist also nur der Motor-PT1. Die Differenz von 20 ms ist die
% fehlende Messkette und steht deshalb hier.
%
% Was die 20 ms bringen (S-4-Replay gegen den Flug, e_z Spitze-Spitze im
% Halteflug): 5 ms -> 11.6 cm, 25 ms -> 16.2 cm, Flug 25.1 cm. Also naeher,
% aber nicht deckungsgleich -- der Rest ist noch NICHT erklaert.
%
% Ausdruecklich NICHT erklaert: das 0.56-Hz-Klingeln der x-Achse im Flug. Im
% Sweep taucht es in der Simulation erst bei 80 ms auf, und dort ist e_z mit
% 40.9 cm schon weit ueber dem Flug. Latenz allein ist also nicht die Ursache.
%
% Vorbehalt: die 20 ms sind eine SUMME und koennten teilweise aktuatorseitig
% sitzen (ESC-Eingangsfilter, PWM-Rahmenrate). Fuer die Schleifendynamik ist
% das egal -- eine Totzeit wirkt an jeder Stelle im Kreis gleich --, fuer die
% Zuordnung nicht. Der S-1-Waagentest kann den Aktuatoranteil trennen.
link_flat_params.latency = 25e-3; % s
link_flat_params.N_delay = round(link_flat_params.latency / Ts_inner);
assert(abs(link_flat_params.N_delay*Ts_inner - link_flat_params.latency) < 1e-12, ...
    'link_flat.latency ist kein ganzzahliges Vielfaches von Ts_inner!');

% --- Frischrate der Zustandsdaten (Durchsatzgrenze, KEIN Verlust) -------------
% Gemessen im Flug 05.08.2026 (dritte S-4-Aufzeichnung): von 1951 Slow-Takten
% trugen 635 (32.5 %) exakt denselben mocap_pos wie der Vortakt. Effektiv also
% 67.5 Hz frische Zustandsdaten statt 100 Hz. Das ist kein Paketverlust: die
% Lauflaengen waren 462x1 / 58x2 / 7x>=3, bei Bernoulli mit derselben Rate waeren
% 355 / 116 / 56 zu erwarten -- viel zu wenige lange Bloecke, das Muster ist
% regelmaessiger als Zufall. Auch keine Quantisierung (1 LSB = 0.61 mm, ein
% frischer Schritt im Mittel 1.8 mm) und nicht bewegungsabhaengig (am Boden
% 31.3 %, im Flug 32.8 %).
% Ursache vermutlich der 2-Frame-Split: der Assembler in mcu_flat_packet.hpp gibt
% cmd erst frei, wenn A UND B mit gleicher seq da sind, und mocap_pos/q_ext
% liegen in A. Zwei 27-B-Frames bei 100 Hz sind 200 Frames/s; 67.5 Paare/s
% entsprechen ~135 Frames/s.
%
% Was es KOSTET (S-4-Replay, e_z Spitze-Spitze im Halten): 100 Hz -> 12.4 cm,
% 66.7 Hz -> 13.4 cm, 50 Hz -> 13.9 cm. Also gut 1 cm bei der gemessenen Rate.
% Was es NICHT erklaert: die 0.46-Hz-Schwingung des Flugs (22.0 cm). Die taucht
% in der Simulation auch bei 50 Hz nicht auf, das Spektrum bleibt ohne Peak.
% Die halbierte Zustandsrate ist also ein eigener, kleiner Defekt -- nicht die
% Ursache des Hoehenproblems.
% hold_gcs = 3 haelt jeden dritten GCS-Takt -> 33.3 % stehend, 66.7 Hz frisch.
% hold_gcs = 0 schaltet es ab (Zustand vor dem 05.08.2026).
% link_tx_flat laeuft mit Ts_inner, nicht mit Ts_gcs -- deshalb wird die Vorgabe
% hier in Aufrufe umgerechnet, damit ein ganzer GCS-Takt am Stueck steht.
hold_gcs   = 3;
n_per_gcs  = round(Ts_gcs / Ts_inner);
link_flat_params.hold_gcs    = hold_gcs;
link_flat_params.hold_period = hold_gcs * n_per_gcs;   % Periode in Aufrufen
link_flat_params.hold_span   = (hold_gcs > 0) * n_per_gcs;  % davon gehalten

% --- Paketverlust ---
link_flat_params.pdrop = 0.02;
link_flat_params.seed  = uint32(24680);   % eigener Seed (!= Kaskaden-Link)

% --- int16-Quantisierung ------------------------------------------------------
% Skalen je Element; Reihenfolge wie oben. lsb = fs/qmax.
link_flat_params.qmax = int16(32767);
link_flat_params.qmin = int16(-32768);
link_flat_params.fs = [ 20; 20; 20; ...      % mocap_pos [m]    (lsb 0.6 mm)
                        20; 20; 20; ...      % p_ref     [m]
                        20; 20; 20; ...      % v_ref     [m/s]
                        50; 50; 50; ...      % a_ref     [m/s^2]
                       200;200;200; ...      % j_ref     [m/s^3]
                      2000;2000;2000; ...    % s_ref     [m/s^4]
                         4; 20; 200 ];       % yaw_ref   [rad; rad/s; rad/s^2]

% --- Init-Pakete (= Hover am Startpunkt) --------------------------------------
scal_init = [ quadcop.x0; quadcop.x0; zeros(15,1) ];   % mocap=p_ref=x0, Rest 0
lsb_link  = double(link_flat_params.fs) / double(link_flat_params.qmax);
link_flat_params.pkt_init = int16( min(max(round(scal_init ./ lsb_link), ...
                        double(link_flat_params.qmin)), double(link_flat_params.qmax)) );

% q_ext = Identitaet -> smallest-three-Code (uint32).
link_flat_params.q_init = pack_quat_sm3([1;0;0;0]);

link_flat_params.flags_init = [uint8(0); boolean(0)]; % [estop=0; ack=false]

% --- Delay-Buffer-ICs (InputProcessing='Elements as channels (sample based)') --
link_flat_params.pkt_init_delay   = repmat(link_flat_params.pkt_init,   [1, 1, link_flat_params.N_delay]);
link_flat_params.q_init_delay     = repmat(link_flat_params.q_init,     [1, 1, link_flat_params.N_delay]);
link_flat_params.flags_init_delay = repmat(link_flat_params.flags_init, [1, 1, link_flat_params.N_delay]);
end
