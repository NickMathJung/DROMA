function R = mount_from_acc(acc_rest)
%mount_from_acc  R_mount aus einer Ruhe-Accel-Messung berechnen.
%   R = align(acc_rest -> [0 0 g]): die minimale Drehung, die den gemessenen
%   Ruhe-Beschleunigungsvektor (Motoren aus, Drohne mocap-level, still) auf die
%   Hochachse [0 0 1] dreht. Ergebnis ist der Montage-Offset R_mount, der in
%   drone_hal.cpp in die MOUNT[id]-Tabelle eingetragen wird (nach R_bs angewandt).
%
%   WICHTIG: acc_rest muss im Body-Frame NACH R_bs, aber MIT Identitaets-MOUNT
%   gemessen sein -> also die betreffende MOUNT[id] vor der Messung auf Identitaet
%   flashen und den Ruhe-acc aus dem [tick]-Report mitteln (mehrere Sekunden).
%
%   Eingang:  acc_rest (3x1) [m/s^2]  gemittelter Ruhe-Accel im Body-Frame
%   Ausgang:  R (3x3)                 R_mount, mit R*acc_rest ~ [0;0;|acc|]
arguments (Input)
    acc_rest (3,1) double
end
    a = acc_rest / norm(acc_rest);      % Ist-Richtung (Einheit)
    t = [0; 0; 1];                      % Soll: Schwerkraft auf +z
    v = cross(a, t);
    s = norm(v);
    c = dot(a, t);
    if s < 1e-9
        R = eye(3);                     % schon ausgerichtet (oder exakt entgegengesetzt)
    else
        vx = [   0  -v(3)  v(2);
              v(3)     0  -v(1);
             -v(2)  v(1)     0 ];       % skew(v)
        R = eye(3) + vx + vx*vx * ((1 - c) / s^2);   % Rodrigues
    end

    % Kontrolle + Rest-Neigung
    a_corr = R * acc_rest;
    tilt_before = acosd(min(1, a(3)));
    tilt_after  = acosd(min(1, a_corr(3)/norm(a_corr)));
    fprintf('Neigung vorher %.2f deg -> nachher %.2f deg\n', tilt_before, tilt_after);

    % Als C-Block fuer die MOUNT[id]-Tabelle in drone_hal.cpp ausgeben
    fprintf('    {{ % .9f, % .9f, % .9f},\n', R(1,1), R(1,2), R(1,3));
    fprintf('     { % .9f, % .9f, % .9f},\n', R(2,1), R(2,2), R(2,3));
    fprintf('     { % .9f, % .9f, % .9f}},\n', R(3,1), R(3,2), R(3,3));
end
