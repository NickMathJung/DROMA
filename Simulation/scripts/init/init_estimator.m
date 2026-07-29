function [mahony,luen] = init_estimator(Ts_gcs)
%init_estimator initializes the mahony filter gains and the gain for the
%   discerete Luenberger observer for the estimation of the velocity
arguments (Input)
    Ts_gcs (1,1) double % sample time of ground station -y used for discretization of Luenberger observer
end

arguments (Output)
    mahony struct % holding gains for the mahony filter
    luen struct % holding gains and matrices for the Luenberger observer
end

% Mahony-Komplementaerfilter
mahony.ka = 1.0;
mahony.kE = 25.0;
mahony.q_init = angle2quat(deg2rad(0), 0, deg2rad(0))';   % ZYX, scalar-first

% Luenberger für Translation, kontinuierlicher Entwurf, achsweise Doppelintegrator
%
% Continuous-time model used by the observer
%   d/dt [p; v] = A*[p;v] + B*a_cmd
%   y           = C*[p;v]                       (position measurement)
% with a_cmd the commanded inertial acceleration.
%
% Observer:
%   d/dt xi = A*xi + B*a_cmd + L*(y - C*xi)
luen.poles = 2.5*[-10 -10 -10 -10 -10 -10];
luen.A = [zeros(3) eye(3); zeros(3) zeros(3)];
luen.B = [zeros(3); eye(3)];
luen.C = [eye(3) zeros(3)];
pa = luen.poles(1:2:end);                 
pb = luen.poles(2:2:end);                 
% Achsweise: Pol-Paar (pa,pb) -> l1 = -(pa+pb), l2 = pa·pb (s² + l1 s + l2 = (s-pa)(s-pb))
luen.L = [diag(-(pa+pb)); diag(pa.*pb)];  % 6x3: [Lp; Lv]
eigs_translational_obsv = eig(luen.A - luen.L * luen.C);
fprintf('Observer continuous-time poles:\n'); disp(sort(real(eigs_translational_obsv)).');

% ----- Exact (ZOH) discretization of the observer ---------------------------
% xi_{k+1} = Ad * xi_k + Bd_u * a_cmd_k + Bd_y * y_k
%
% The closed-loop observer is LTI with input [a_cmd; y] and dynamics
%   d/dt xi = (A - L*C) xi + [B  L] * [a_cmd; y]
A_obs_c = luen.A - luen.L*luen.C;
B_obs_c = [luen.B, luen.L];
sys_c   = ss(A_obs_c, B_obs_c, eye(6), 0);
sys_d   = c2d(sys_c, Ts_gcs, 'zoh');
luen.Ad      = sys_d.A;
luen.Bd      = sys_d.B;
luen.Cd      = sys_d.C;
luen.Dd      = sys_d.D;
luen.Bd_u    = luen.Bd(:,1:3); % column block for a_cmd
luen.Bd_y    = luen.Bd(:,4:6); % column block for position measurement

% Discrete-time eigenvalues — useful sanity check
fprintf('Discrete-time observer eigenvalues |z| (ZOH, dt=%.4g):\n', Ts_gcs);
disp(sort(abs(eig(luen.Ad))).');
end