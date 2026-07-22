function q = unpack_quat_sm3(code)
%#codegen
% unpack_quat_sm3  Umkehrung von pack_quat_sm3: uint32 -> Quaternion (4x1)
%   Rekonstruiert die weggelassene groesste Komponente aus |q|=1 (positiv)
%   danach Renormierung. Bit-identisch zum C++-Codec.
%   Sonderfall: code 0 ist reserviert und bedeutet "kein gueltiger Lagebezug";
%   dann kommt ein Null-Quaternion zurueck, das der Mahony-Guard abfaengt.

code = uint32(code);

if code == 0
    q = zeros(4,1);
    return;
end

imax = double(bitshift(code, -30)) + 1; % 1..4
u = zeros(3,1);
u(1) = double(bitand(bitshift(code, -20), uint32(1023)));
u(2) = double(bitand(bitshift(code, -10), uint32(1023)));
u(3) = double(bitand(code,                uint32(1023)));

SCALE = 511 * sqrt(2);
c = (u - 512) / SCALE; % in [-1/sqrt2, 1/sqrt2]

q = zeros(4,1);
k = 1;  ssum = 0;
for i = 1:4
    if i ~= imax
        q(i) = c(k);
        ssum = ssum + c(k)*c(k);
        k = k + 1;
    end
end
q(imax) = sqrt(max(0, 1 - ssum)); % groesste, positiv

n = sqrt(q(1)*q(1) + q(2)*q(2) + q(3)*q(3) + q(4)*q(4));
if n < 1e-12
    q = [1;0;0;0];
else
    q = q / n;
end
end
