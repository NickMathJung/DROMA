function [kill, fault_src, dbg] = safety_overspeed(gyro_corr, q_hat, estop, ack, btn, safety)
%#codegen
% safety_overspeed  Onboard-Kill-Latch mit vier Quellen.
%
% Setzt ein latchendes kill-Flag, sobald eine der vier Fehlerbedingungen
% zutrifft. Ein nachgelagerter Switch zwingt daraufhin rotors_cmd=0 (nach dem
% Mixer, vor Motor-PT1 und ESC). Der Kill hat Vorrang vor Land und wird in der
% Luft nicht zurueckgenommen. Die vier Quellen teilen sich einen Latch, eine
% Aktion und eine Re-Arm-Bedingung, damit die Logik an einer Stelle steht:
%   1 Overspeed : |gyro| > omega_max, ueber debounce_N Samples entprellt.
%   2 Hard-Kill : estop==2 (Uplink oder Link-Watchdog), sofort.
%   3 Tilt      : Kippwinkel > tilt_max, ueber tilt_debounce_N Samples entprellt.
%   4 Taster    : steigende Flanke von btn (lokaler Teensy-Taster). Damit kann
%                 der Bediener die Motoren vor Ort sicher stilllegen, bevor er
%                 den Akku absteckt.
% Der geregelte Soft-Land-Fall (estop==1) gehoert nicht hierher, sondern in die
% Mode-Maschine der GCS.
%
% Eingaenge
%   gyro_corr : 3x1  bias-korrigierte Drehrate [rad/s] (Messung, kein Schaetzer)
%   q_hat     : 4x1  geschaetzte Lage, scalar-first [w x y z] (fuer den Tilt)
%   estop     : uint8  0 normal / 1 soft-land / 2 hard-kill (aus Bus_Cmd, Uplink)
%   ack       : bool   Quittung, NUR Bus_Cmd.ack (Uplink) -> loest den Latch
%   btn       : bool   lokaler Teensy-Taster (active-high); Flanke -> Kill
%   safety    : struct  .omega_max [rad/s], .debounce_N (>=1), .use_norm (bool),
%                       .tilt_cos_min (= cos(tilt_max)), .tilt_debounce_N (>=1)
%
% Ausgaenge
%   kill      : bool   latched -> Switch downstream setzt rotors_cmd=0
%   fault_src : uint8  0 keine / 1 overspeed / 2 hard-kill / 3 tilt / 4 taster
%   dbg       : 3x1    [cnt; over_inst; ack_edge] (verify/logging)
%
% Re-Armen (Fault -> Armed) nur bei steigender ack-Flanke aus Bus_Cmd und nur,
% wenn gerade keine Fehlerbedingung anliegt: kein Overspeed, kein zu grosser
% Kippwinkel, kein Hard-Kill und der Taster nicht gedrueckt. Ausgewertet wird die
% ack-Flanke, nicht der Pegel, damit ein gehaltenes ack weder einen frischen Trip
% sofort loescht noch mitten im Flug re-armt. Der Taster ist mit Absicht getrennt
% vom ack: er tut nichts mehr fuers Quittieren, sondern loest nur noch aus, und er
% blockiert das Re-Armen, solange er gehalten wird. So drehen die Propeller beim
% Akkuwechsel garantiert nicht an. Zur verworfenen Interlock-Variante siehe die
% Notiz am Dateiende.

persistent latched cnt tcnt ack_prev btn_prev src
if isempty(latched)
    latched  = false;
    cnt      = uint16(0);
    tcnt     = uint16(0);
    ack_prev = false;
    btn_prev = false;
    src      = uint8(0);
end

gw = reshape(gyro_corr, 3, 1);

% --- Overspeed-Detektor, entprellt (N aufeinanderfolgende Samples) ---
if safety.use_norm
    over_inst = sqrt(gw(1)*gw(1) + gw(2)*gw(2) + gw(3)*gw(3)) > safety.omega_max;
else
    over_inst = (abs(gw(1)) > safety.omega_max) || ...
                (abs(gw(2)) > safety.omega_max) || ...
                (abs(gw(3)) > safety.omega_max);
end

Nreq = uint16(safety.debounce_N);
if over_inst
    if cnt < Nreq
        cnt = cnt + uint16(1);
    end
else
    cnt = uint16(0);                 % ein gutes Sample setzt den Zaehler zurueck
end
over_deb = cnt >= Nreq;

% --- Tilt-Detektor, entprellt ---
% Kippwinkel gegen die Vertikale aus der geschaetzten Lage:
%   cos(tilt) = R33 = (w^2 - x^2 - y^2 + z^2) / |q|^2.
% Die Normierung faengt ein leicht denormiertes q_hat ab; ein degeneriertes
% (Null-)Quaternion gilt als level (cos=1), damit kein Fehltrip entsteht.
q  = reshape(q_hat, 4, 1);
n2 = q(1)*q(1) + q(2)*q(2) + q(3)*q(3) + q(4)*q(4);
if n2 < 1e-12
    tilt_inst = false;
else
    cos_tilt  = (q(1)*q(1) - q(2)*q(2) - q(3)*q(3) + q(4)*q(4)) / n2;
    tilt_inst = cos_tilt < safety.tilt_cos_min;
end

Ntilt = uint16(safety.tilt_debounce_N);
if tilt_inst
    if tcnt < Ntilt
        tcnt = tcnt + uint16(1);
    end
else
    tcnt = uint16(0);
end
tilt_deb = tcnt >= Ntilt;

% --- Hard-Kill: sofort, keine Entprellung ---
hard_kill = (estop == uint8(2));

% --- Taster: steigende Flanke ---
btn_edge = btn && ~btn_prev;

% --- KILL setzen (latcht; Quelle nur beim ersten Setzen vermerkt) ---
if over_deb && ~latched
    latched = true;
    src = uint8(1);                  % Quelle: Overspeed
end
if hard_kill && ~latched
    latched = true;
    src = uint8(2);                  % Quelle: Hard-Kill
end
if tilt_deb && ~latched
    latched = true;
    src = uint8(3);                  % Quelle: Tilt
end
if btn_edge && ~latched
    latched = true;
    src = uint8(4);                  % Quelle: lokaler Taster
end

% Re-Arm: steigende ack-Flanke, und keine Fehlerbedingung liegt gerade an.
ack_edge = ack && ~ack_prev;
if latched && ack_edge && ~over_inst && ~tilt_inst && (estop ~= uint8(2)) && ~btn
    latched = false;
    cnt     = uint16(0);
    tcnt    = uint16(0);
    src     = uint8(0);
end
ack_prev = ack;
btn_prev = btn;

kill      = latched;
fault_src = src;
dbg       = [double(cnt); double(over_inst); double(ack_edge)];
end

% -------------------------------------------------------------------------
% Verworfene Variante: Arming-Idle-Interlock. Getestet und wieder rausgeworfen,
% bitte nicht ohne neue Argumente zurueckbauen. Sie verlangte fuers Re-Armen
% zusaetzlich F_des <= safety.F_rearm_idle (= 0.1*m*g).
%
% Aus dem F_des-Sweep gegen mcu.slx (level, gyro_corr=0):
%   - Die Schwelle griff bit-exakt bei 0.946665 N (<= inklusiv).
%   - Der throttle im Loese-Tick ist aber nicht 0: polyval(p_from_omega_sq, 0)
%     = 8.404 % (konstanter Term). Bei OneShot125 sind das ~555 counts / 135 us,
%     also ueber der Anlaufschwelle (~5-10 %) — die Props laufen beim Re-Armen
%     ohnehin an. "Schub runter zum Armen" macht das Loesen nicht motorfrei.
%   - Der Gewinn lag bei 9.94 % statt 23.43 % throttle, also 13.5 Punkten.
% Der lokale Taster (Pin 21) ist heute selbst eine Kill-Quelle (siehe oben) und
% blockiert das Re-Armen, solange er gehalten wird; damit ist das urspruengliche
% Ziel "am Boden sicher loesen" ohne den Idle-Interlock erreicht.
