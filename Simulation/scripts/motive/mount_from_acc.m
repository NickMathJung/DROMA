function R = mount_from_acc(acc_rest)
%mount_from_acc  Montage-Offset R_mount aus einer Ruhe-Accel-Messung berechnen.
%   Minimale Drehung, die den gemessenen Ruhe-Beschleunigungsvektor auf die
%   Hochachse [0 0 1] dreht. acc_rest wird im Body-Frame nach R_bs und mit
%   Identitaets-MOUNT gemessen.
%
%   Eingang:  acc_rest (3x1)  gemittelter Ruhe-Accel im Body-Frame
%   Ausgang:  R (3x3)         R_mount, mit R*acc_rest ~ [0;0;|acc|]
arguments (Input)
    acc_rest (3,1) double
end
    a = acc_rest / norm(acc_rest); % Ist-Richtung normiert
    t = [0; 0; 1]; % Soll: Schwerkraft auf +z
    v = cross(a, t);
    s = norm(v);
    c = dot(a, t);
    if s < 1e-9
        R = eye(3); % schon ausgerichtet
    else
        vx = [   0  -v(3)  v(2);
              v(3)     0  -v(1);
             -v(2)  v(1)     0 ]; % skew(v)
        R = eye(3) + vx + vx*vx * ((1 - c) / s^2); % Rodrigues
    end

    % Kontrolle + Rest-Neigung
    a_corr = R * acc_rest;
    tilt_before = acosd(min(1, a(3)));
    tilt_after  = acosd(min(1, a_corr(3)/norm(a_corr)));
    fprintf('Neigung vorher %.2f deg -> nachher %.2f deg\n', tilt_before, tilt_after);

    % Als C-Block fuer die MOUNT[id]-Tabelle ausgeben
    fprintf('    {{ % .9f, % .9f, % .9f},\n', R(1,1), R(1,2), R(1,3));
    fprintf('     { % .9f, % .9f, % .9f},\n', R(2,1), R(2,2), R(2,3));
    fprintf('     { % .9f, % .9f, % .9f}},\n', R(3,1), R(3,2), R(3,3));
end
