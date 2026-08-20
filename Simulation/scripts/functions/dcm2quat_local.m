function q = dcm2quat_local(R)
%#codegen
% Wandelt eine Drehmatrix nach der Shepperd-Methode in ein Quaternion um.
% Rueckgabe q = [q0;q1;q2;q3] (Spalte), normiert.
tr = R(1,1) + R(2,2) + R(3,3);
if tr > 0
    S  = 2*sqrt(tr + 1);
    q0 = 0.25*S;
    q1 = (R(2,3) - R(3,2))/S;
    q2 = (R(3,1) - R(1,3))/S;
    q3 = (R(1,2) - R(2,1))/S;
elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
    S  = 2*sqrt(1 + R(1,1) - R(2,2) - R(3,3));
    q0 = (R(2,3) - R(3,2))/S;
    q1 = 0.25*S;
    q2 = (R(1,2) + R(2,1))/S;
    q3 = (R(1,3) + R(3,1))/S;
elseif R(2,2) > R(3,3)
    S  = 2*sqrt(1 + R(2,2) - R(1,1) - R(3,3));
    q0 = (R(3,1) - R(1,3))/S;
    q1 = (R(1,2) + R(2,1))/S;
    q2 = 0.25*S;
    q3 = (R(2,3) + R(3,2))/S;
else
    S  = 2*sqrt(1 + R(3,3) - R(1,1) - R(2,2));
    q0 = (R(1,2) - R(2,1))/S;
    q1 = (R(1,3) + R(3,1))/S;
    q2 = (R(2,3) + R(3,2))/S;
    q3 = 0.25*S;
end
q = [q0; q1; q2; q3];
q = q / norm(q);
end