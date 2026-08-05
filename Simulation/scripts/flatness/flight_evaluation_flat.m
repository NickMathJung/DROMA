% flight_evaluation_flat  Tracking-Auswertung eines bench_flat.slx-Laufs.


t_flight   = out.tout;
x          = squeeze(out.mocap_pos.Data);
x_ref      = squeeze(out.x_ref.Data);
v_ref      = squeeze(out.v_ref.Data);
mocap_quat = squeeze(out.mocap_quat.Data)';

% --- Referenz um T_lead zurueckschieben -----------------------
if ~exist('fctrl','var')
    error(['fctrl fehlt im Workspace (params.m nicht gelaufen?) -- ' ...
           'ohne fctrl.T_lead ist die Referenz nicht in den Zeitplan rueckbar.']);
end
dt_ref = median(diff(out.x_ref.Time));
n_lead = round(fctrl.T_lead / dt_ref);
if n_lead > 0
    shift_back = @(A) shift_time_dim(A, n_lead, numel(out.x_ref.Time));
    x_ref = shift_back(x_ref);
    v_ref = shift_back(v_ref);
end

e_p = x - x_ref;
norm_e_p = vecnorm(e_p,2,2);
plot(t_flight, norm_e_p);
title("Norm of the tracking error $\|p - p_s\|_2$", 'Interpreter','latex');
xlabel("t in [s]");
ylabel("$\|p - p_s\|_2$", 'Interpreter','latex');
zielOrdner = 'C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\data';
dateiname_tr_err = 'norm_e_p.mat';
dateiname_x = 'x.mat';
dateiname_x_ref = 'x_ref.mat';
dateiname_v_ref = 'v_ref.mat';
dateiname_mocap_quat = 'mocap_quat.mat';

save(fullfile(zielOrdner, dateiname_tr_err), 'norm_e_p');
save(fullfile(zielOrdner, dateiname_x), 'x');
save(fullfile(zielOrdner, dateiname_x_ref), 'x_ref');
save(fullfile(zielOrdner, dateiname_v_ref), 'v_ref');
save(fullfile(zielOrdner, dateiname_mocap_quat), 'mocap_quat');
save(fullfile(zielOrdner,'t_flight.mat'),'t_flight');

function A = shift_time_dim(A, n, nt)
% Signal um n Abtastungen nach SPAET schieben (= Referenz zurueck in den
% Zeitplan); Anfang mit dem ersten Wert auffuellen, Laenge bleibt gleich.
if size(A,1) == nt
    A = [repmat(A(1,:), n, 1); A(1:end-n, :)];
elseif size(A,2) == nt
    A = [repmat(A(:,1), 1, n), A(:, 1:end-n)];
else
    error('Zeitachse (%d Punkte) passt zu keiner Dimension von %s.', nt, mat2str(size(A)));
end
end
