function flight_evaluation(id)
%flight_evaluation  Auswertung eines Flugversuchs von Drohne id, Plot der
%   Trackingfehler.
arguments
    id (1,1) double = 1
end
out        = evalin('base', 'out');
controller = evalin('base', 'controller');
ids = evalin('base', 'mocap.streaming_ids');
d = find(ids == id, 1);
assert(~isempty(d), 'flight_evaluation: id=%d nicht in mocap.streaming_ids %s.', ...
    id, mat2str(ids));

g = @(name) orient_ts(out.get(sprintf('%s_%d', name, d)).Data, numel(out.tout));
t_flight   = out.tout;
x          = g('mocap_pos');
x_ref      = g('x_ref');
v_hat      = g('v_hat');
v_ref      = g('v_ref');
q_des      = g('q_des');
F_des      = g('F_des').';
mocap_quat = g('mocap_quat');

% ------- Referenz um T_lead zurueckschieben ------
dt_ref = median(diff(t_flight));
n_lead = round(controller.T_lead / dt_ref);
if n_lead > 0
    x_ref = [repmat(x_ref(1,:), n_lead, 1); x_ref(1:end-n_lead, :)];
    v_ref = [repmat(v_ref(1,:), n_lead, 1); v_ref(1:end-n_lead, :)];
end

e_p = x - x_ref;
e_p_x = x(:,1) - x_ref(:,1);
e_p_y = x(:,2) - x_ref(:,2);
e_p_z = x(:,3) - x_ref(:,3);
norm_e_p = vecnorm(e_p,2,2);
norm_e_p_x = vecnorm(e_p_x,2,2);
norm_e_p_y = vecnorm(e_p_y,2,2);
norm_e_p_z = vecnorm(e_p_z,2,2);
figure('Name', sprintf('Drohne id=%d', id));
plot(t_flight, norm_e_p_x);
hold on
plot(t_flight, norm_e_p_y);
plot(t_flight, norm_e_p_z);
title(sprintf(['Norm of the individual tracking errors per axis ' ...
    '$\\|p_i - p_{s,i}\\|_2$, $i=x,y,z$ (drone id=%d)'], id), 'Interpreter','latex');
xlabel("t in [s]");
ylabel("$\|p_i - p_{s,i}\|_2$", 'Interpreter','latex');
legend("$\|p_x - p_{s,x}\|_2$", "$\|p_y - p_{s,y}\|_2$", "$\|p_z - p_{s,z}\|_2$", 'Interpreter','latex');

% ------- Speichern ---
zielOrdner = 'C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\data';
sfx = sprintf('_id%d', id);
save(fullfile(zielOrdner, ['norm_e_p'   sfx '.mat']), 'norm_e_p');
save(fullfile(zielOrdner, ['x'          sfx '.mat']), 'x');
save(fullfile(zielOrdner, ['x_ref'      sfx '.mat']), 'x_ref');
save(fullfile(zielOrdner, ['v_hat'      sfx '.mat']), 'v_hat');
save(fullfile(zielOrdner, ['v_ref'      sfx '.mat']), 'v_ref');
save(fullfile(zielOrdner, ['q_des'      sfx '.mat']), 'q_des');
save(fullfile(zielOrdner, ['F_des'      sfx '.mat']), 'F_des');
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
