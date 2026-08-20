function r = quatConj(a)
%#codegen
% Konjugiertes Quaternion zu a.
    r = [a(1); -a(2); -a(3); -a(4)];
end