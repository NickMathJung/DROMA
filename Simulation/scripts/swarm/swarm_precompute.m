function ref = swarm_precompute(kappa, p0_drones, cfg_extra)
%swarm_precompute - Referenzen fuer den Schwarmflug erzeugen.
%   ref = swarm_precompute() - Standardlauf ohne Mocap (p0 = [])
%   ref = swarm_precompute(1.3, read_swarm_origins([1 2])) - mit Startposen
%
%   Ruft main_DROMA (Containment-Repo), streckt die Zeit um kappa
%   (v/kappa, a/kappa^2, j/kappa^3 -- macht den Einschwing-Transienten
%   fliegbar, ohne die MAS-Dynamik anzufassen), tastet auf das Ts_gcs Zeitraster
%   ab, prüft Käfiggrenzen und schreibt data\swarm_ref.mat für bench.slx/gcu.slx.
%   Die Bench-InitFcn lädt nur die .mat.
arguments
    kappa     (1,1) double = 1.45
    p0_drones double = []
    cfg_extra struct = struct()
end

CONTAINMENT = ['C:\Users\Rakete\Documents\Drohnenversuchsstand\' ...
    'hyperbolic-2d-containment-control-with-bezier-surfaces\heterogeneous'];
DROMA_DATA = 'C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\data';
% Kaefig-Nutzvolumen (Nick, 13.08.2026; Sicherheitsrand schon enthalten).
% z-Untergrenze 0: der Bodenstart der Agenten ist gewollt.
CAGE = struct('x', [-1.9 1.9], 'y', [-1 1], 'z', [0 2.8]);
% Envelope der geflogenen Box (3.1-cm-Flug): max|v| 2.46, max|a| 6.24
VMAX = 2.4; AMAX = 6.2;
Ts = 0.01; % Ts_gcs

addpath(CONTAINMENT);
cfg = cfg_extra;
cfg.p0_drones = p0_drones;
% Standard-Auslegung "Kuppel" (13.08.2026): Leader-Quader z 0.8..3.0 (zentriert,
% z_offset = Mitte), um y gekippt + 180-deg-Flip -> Sattelbauch woelbt sich NACH
% OBEN, Ecken unten auf EINER Hoehe (deshalb ist die Rotation downwash-frei).
% Start senkrecht unter der eigenen Flaechenposition; omega=0.25 kreist die
% Kuppel um die eigene Hochachse. Alles per cfg_extra uebersteuerbar.
% Zieldynamik (s+2.5)^2 + Randpole -3.3..-2.5: steif genug, dass die Formation
% der Rotation bis omega=1 folgt (|G(j1)|=0.86; weich (s+1.5)^2 schrumpfte auf
% 69 % -> Abstands-Assert), weich genug fuer den Einschwing-Transienten (kappa 1.45).
SAIL = struct('extent', [1.2 2.6 2.2], 'R_L', [0 0 1; 0 -1 0; 1 0 0], ...
              'z_offset', 1.9, 'center_leaders', true, ...
              'ground_under_agent', true, 'omega', 1.0, ...
              'a1', 5, 'b1', 6.25, 'poles_z', linspace(-3.3, -2.5, 18));
fn = fieldnames(SAIL);
for k = 1:numel(fn)
    if ~isfield(cfg, fn{k}), cfg.(fn{k}) = SAIL.(fn{k}); end
end
% Agenten-Defaults der Segel-Geometrie (alle Paare downwashfrei)
if ~isfield(cfg, 'agents') || isempty(cfg.agents)
    AG = {[], [1 1; 20 9], [1 1; 20 1; 1 9], [1 1; 20 1; 1 9; 20 9]};
    nD = max(2, size(p0_drones, 2));
    assert(nD <= 4, 'swarm_precompute: fuer n > 4 agents explizit angeben.');
    cfg.agents = AG{nD};
end
res = main_DROMA(cfg);

% --- kappa-Zeitstreckung + Abtastung auf das GCS-Raster -----------------------
T_end = res.t_sim(end) * kappa;
t = (0:Ts:T_end).';
n = size(res.refs.p, 3);
ref = struct('t', t, 'kappa', kappa, 'agents', res.refs.agents, ...
             'cfg', res.cfg, 'tf', res.tf);
for d = 1:n
    ts = res.t_sim * kappa;   % gestreckte Zeitachse
    ref.p(:,:,d) = interp1(ts, res.refs.p(:,:,d), t, 'pchip');
    ref.v(:,:,d) = interp1(ts, res.refs.v(:,:,d), t, 'pchip') / kappa;
    ref.a(:,:,d) = interp1(ts, res.refs.a(:,:,d), t, 'pchip') / kappa^2;
    ref.j(:,:,d) = interp1(ts, res.refs.j(:,:,d), t, 'pchip') / kappa^3;
end

% --- Checks: Kaefig je Agent + Envelope ---------------------------------------
ok = true;
for d = 1:n
    P = ref.p(:,:,d); V = ref.v(:,:,d); A = ref.a(:,:,d);
    lim = [min(P).', max(P).'];
    vmax = max(vecnorm(V,2,2)); amax = max(vecnorm(A,2,2));
    inCage = lim(1,1) >= CAGE.x(1) && lim(1,2) <= CAGE.x(2) && ...
             lim(2,1) >= CAGE.y(1) && lim(2,2) <= CAGE.y(2) && ...
             lim(3,1) >= CAGE.z(1) && lim(3,2) <= CAGE.z(2);
    good = inCage && vmax <= VMAX && amax <= AMAX;
    if good, tag = 'OK'; else, tag = 'FEHLER'; end
    fprintf(['[swarm_precompute] Agent %d (%d,%d): x [%5.2f %5.2f] y [%5.2f %5.2f] ' ...
             'z [%4.2f %4.2f] | max|v| %.2f | max|a| %.2f %s\n'], d, ...
        ref.agents(d,1), ref.agents(d,2), lim.', vmax, amax, tag);
    ok = ok && good;
end
% Abstands-/Downwash-Check ueber alle Paare (kappa streckt nur die Zeit)
dmin_all = inf;
for a = 1:n
    for b = a+1:n
        dab  = vecnorm(ref.p(:,:,a) - ref.p(:,:,b), 2, 2);
        dxy  = vecnorm(ref.p(:,1:2,a) - ref.p(:,1:2,b), 2, 2);
        dz   = abs(ref.p(:,3,a) - ref.p(:,3,b));
        fprintf(['[swarm_precompute] Paar %d-%d: Abstand min %.2f m | ' ...
                 'Downwash-Momente (dxy<0.8 & dz>0.3): %d\n'], a, b, ...
                min(dab), nnz(dxy < 0.8 & dz > 0.3));
        dmin_all = min(dmin_all, min(dab));
    end
end
assert(dmin_all > 0.6, 'Agentenabstand < 0.6 m -- Paarwahl/Radius pruefen.');
if ~ok, warning('swarm_precompute:limits', 'Grenzverletzung -- siehe Report oben.'); end

% --- Schlanke Animationsdaten (Agenten + Leader, ohne Beobachterzustaende) ----
nKeep = 2*res.dim_X + res.params.n_L + 1;
anim = struct('t_sim', res.t_sim * kappa, 'y_sim', res.y_sim(:, 1:nKeep), ...
              'params', res.params, 'Pi', res.Pi, 'P_L', res.P_L, ...
              'N', res.N, 'M', res.M, 'dim_X', res.dim_X);

save(fullfile(DROMA_DATA, 'swarm_ref.mat'), 'ref', 'anim', '-v7.3');
fprintf('[swarm_precompute] gespeichert: %s (T = %.1f s, Ts = %g s)\n', ...
    fullfile(DROMA_DATA, 'swarm_ref.mat'), T_end, Ts);
end
