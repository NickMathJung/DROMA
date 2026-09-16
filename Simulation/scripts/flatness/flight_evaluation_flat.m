function flight_evaluation_flat(id)
%flight_evaluation_flat  Tracking-Auswertung eines bench_flat-Laufs fuer Drohne id.
%   Gegenstueck zu flight_evaluation(id) der Kaskade: bench_flat loggt je GCS-Pfad d
%   mocap_pos_d, mocap_quat_d, x_ref_d, v_ref_d, a_ref_d (Pfad d = Position von id in
%   mocap.streaming_ids). Geloggt wird die Referenz x_s(t+T_lead); sie wird hier um
%   T_lead zurueckgeschoben, damit der Fehler auf dem Raum-Zeit-Fahrplan gemessen wird.
%   Ergebnisse: data\*_id<id>.mat (Namen wie bei der Kaskade, zusaetzlich a_ref).
arguments
    id (1,1) double = 1
end
out   = evalin('base', 'out');
fctrl = evalin('base', 'fctrl');
ids   = evalin('base', 'mocap.streaming_ids');
d = find(ids == id, 1);
assert(~isempty(d), 'flight_evaluation_flat: id=%d nicht in mocap.streaming_ids %s.', ...
    id, mat2str(ids));

g = @(name) orient_ts(out.get(sprintf('%s_%d', name, d)).Data, numel(out.tout));
t_flight   = out.tout;
x          = g('mocap_pos');
x_ref      = g('x_ref');
v_ref      = g('v_ref');
a_ref      = g('a_ref');
mocap_quat = g('mocap_quat');

% --- Referenz um T_lead zurueckschieben -----------------------
dt_ref = median(diff(t_flight));
n_lead = round(fctrl.T_lead / dt_ref);
if n_lead > 0
    x_ref = [repmat(x_ref(1,:), n_lead, 1); x_ref(1:end-n_lead, :)];
    v_ref = [repmat(v_ref(1,:), n_lead, 1); v_ref(1:end-n_lead, :)];
    a_ref = [repmat(a_ref(1,:), n_lead, 1); a_ref(1:end-n_lead, :)];
end

e_p = x - x_ref;
norm_e_p   = vecnorm(e_p, 2, 2);
norm_e_p_x = abs(e_p(:,1));
norm_e_p_y = abs(e_p(:,2));
norm_e_p_z = abs(e_p(:,3));
figure('Name', sprintf('Drohne id=%d (flat)', id));
plot(t_flight, norm_e_p_x);
hold on
plot(t_flight, norm_e_p_y);
plot(t_flight, norm_e_p_z);
title(sprintf(['Norm of the individual tracking errors per axis ' ...
    '$\\|p_i - p_{s,i}\\|_2$, $i=x,y,z$ (drone id=%d)'], id), 'Interpreter', 'latex');
xlabel("t in [s]");
ylabel("$\|p_i - p_{s,i}\|_2$", 'Interpreter', 'latex');
legend("$\|p_x - p_{s,x}\|_2$", "$\|p_y - p_{s,y}\|_2$", "$\|p_z - p_{s,z}\|_2$", 'Interpreter', 'latex');
fprintf('[flight_evaluation_flat] id=%d: RMS |e_p| = %.1f mm, max |e_p| = %.1f mm\n', ...
    id, 1e3*sqrt(mean(norm_e_p.^2)), 1e3*max(norm_e_p));

% ------- Speichern ---
zielOrdner = 'C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\data';
if ~isfolder(zielOrdner)
    zielOrdner = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'data');
end
sfx = sprintf('_id%d', id);
save(fullfile(zielOrdner, ['norm_e_p'   sfx '.mat']), 'norm_e_p');
save(fullfile(zielOrdner, ['x'          sfx '.mat']), 'x');
save(fullfile(zielOrdner, ['x_ref'      sfx '.mat']), 'x_ref');
save(fullfile(zielOrdner, ['v_ref'      sfx '.mat']), 'v_ref');
save(fullfile(zielOrdner, ['a_ref'      sfx '.mat']), 'a_ref');
save(fullfile(zielOrdner, ['mocap_quat' sfx '.mat']), 'mocap_quat');
save(fullfile(zielOrdner, ['t_flight'   sfx '.mat']), 't_flight');
end

function A = orient_ts(A, nt)
% Timeseries-Daten auf [nt x k] orientieren.
A = squeeze(A);
if size(A,1) ~= nt && size(A,2) == nt
    A = A.';
end
end
