% flight_evaluation  Tracking-Auswertung eines bench-Laufs (out), Kaskade.
%
% Seit dem Referenz-Vorhalt (controller.T_lead, 06.08.2026) loggt gcu die
% VORGESCHOBENE Referenz. Massstab ist der Raum-Zeitplan: die geloggte
% Referenz wird vor dem Vergleich um T_lead zurueckgeschoben (identisch zu
% flight_evaluation_flat.m, dort steht die Begruendung).

t_flight = out.tout;
x = squeeze(out.mocap_pos.Data);
x_ref = squeeze(out.x_ref.Data)';
v_hat = squeeze(out.v_hat.Data)';
v_ref = squeeze(out.v_ref.Data)';
q_des = squeeze(out.q_des.Data)';
F_des = squeeze(out.F_des.Data)';
mocap_quat = squeeze(out.mocap_quat.Data)';

% ------- Referenz um T_lead zurueckschieben, da Totzeit im System durch zu 
%     frühes senden der Solltrajektorie (100ms) ausgeglichen wird ---------
if ~exist('controller','var') || ~isfield(controller,'T_lead')
    error(['controller.T_lead fehlt im Workspace (params.m nicht gelaufen?) -- ' ...
           'ohne T_lead ist die Referenz nicht in den Zeitplan rueckbar.']);
end
dt_ref = median(diff(out.x_ref.Time));
n_lead = round(controller.T_lead / dt_ref);
if n_lead > 0
    shift_back = @(A) shift_time_dim(A, n_lead, numel(out.x_ref.Time));
    x_ref = shift_back(x_ref);
    v_ref = shift_back(v_ref);
end

e_p = x - x_ref;
e_p_x = x(:,1) - x_ref(:,1);
e_p_y = x(:,2) - x_ref(:,2);
e_p_z = x(:,3) - x_ref(:,3);
norm_e_p = vecnorm(e_p,2,2);
norm_e_p_x = vecnorm(e_p_x,2,2);
norm_e_p_y = vecnorm(e_p_y,2,2);
norm_e_p_z = vecnorm(e_p_z,2,2);
% plot(t_flight, norm_e_p);
plot(t_flight, norm_e_p_x);
hold on
plot(t_flight, norm_e_p_y);
hold on
plot(t_flight, norm_e_p_z);
% title("Norm of the tracking error $\|p - p_s\|_2$", 'Interpreter','latex');
title("Norm of the individual tracking errors per axis $\|p_i - p_{s,i}\|_2$, $i=x,y,z$", 'Interpreter','latex');
xlabel("t in [s]");
% ylabel("$\|p - p_s\|_2$", 'Interpreter','latex');
ylabel("$\|p_i - p_{s,i}\|_2$", 'Interpreter','latex');
legend("$\|p_x - p_{s,x}\|_2$", "$\|p_y - p_{s,y}\|_2$", "$\|p_z - p_{s,z}\|_2$", 'Interpreter','latex');
zielOrdner = 'C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\data';
dateiname_tr_err = 'norm_e_p.mat';
dateiname_x = 'x.mat';
dateiname_x_ref = 'x_ref.mat';
dateiname_v_hat = 'v_hat.mat';
dateiname_v_ref = 'v_ref.mat';
dateiname_q_des = 'q_des.mat';
dateiname_F_des = 'F_des.mat';
dateiname_mocap_quat = 'mocap_quat.mat';

save(fullfile(zielOrdner, dateiname_tr_err), 'norm_e_p');
save(fullfile(zielOrdner, dateiname_x), 'x');
save(fullfile(zielOrdner, dateiname_x_ref), 'x_ref');
save(fullfile(zielOrdner, dateiname_v_hat), 'v_hat');
save(fullfile(zielOrdner, dateiname_v_ref), 'v_ref');
save(fullfile(zielOrdner, dateiname_q_des), 'q_des');
save(fullfile(zielOrdner, dateiname_F_des), 'F_des');
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
