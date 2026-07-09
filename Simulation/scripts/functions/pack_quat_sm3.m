function code = pack_quat_sm3(q)
%#codegen
% pack_quat_sm3  Smallest-three Quaternion-Kompression -> uint32 (32 bit)
%
%   Nutzt |q|=1 und q == -q: die betragsgroesste Komponente (>= 1/sqrt(2))
%   wird weggelassen und aus den drei anderen rekonstruiert.
%
%   Bit-Layout (MUSS mit unpack_quat_sm3.m UND dem C++-Codec uebereinstimmen):
%     [31:30] Index imax der weggelassenen (groessten) Komponente, 0..3
%     [29:20] Komponente a   (Offset-Binary 10-bit)
%     [19:10] Komponente b
%     [ 9: 0] Komponente c
%   je Komponente:  u = qi + 512 ,  qi = clamp(round(c * 511*sqrt(2)), -511, 511)
%
%   Reihenfolge von q (z.B. scalar-first [w x y z]) ist BELIEBIG, aber muss auf
%   Encoder-, Decoder- und C++-Seite IDENTISCH indiziert sein.

q = q(:);
n = sqrt(q(1)*q(1) + q(2)*q(2) + q(3)*q(3) + q(4)*q(4));
if n < 1e-12
    q = [1;0;0;0]; % Fallback fuer degenerierte Eingabe
else
    q = q / n;
end

% betragsgroesste Komponente
imax = 1;  amax = abs(q(1));
for i = 2:4
    if abs(q(i)) > amax
        amax = abs(q(i));  imax = i;
    end
end

% Vorzeichen fixieren: groesste Komponente positiv  (q == -q)
if q(imax) < 0
    q = -q;
end

SCALE = 511 * sqrt(2); % 1/sqrt(2) -> 511
code  = bitshift(uint32(imax-1), 30); % 2-bit Index

slot = 20;
for i = 1:4
    if i ~= imax
        qi = round(q(i) * SCALE);
        if qi >  511, qi =  511; end
        if qi < -511, qi = -511; end
        u    = uint32(qi + 512); % Offset-Binary, [1..1023]
        code = bitor(code, bitshift(u, slot));
        slot = slot - 10;
    end
end
end
