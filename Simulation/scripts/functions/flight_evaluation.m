t_flight = out.trackingErrpr.Time;
e_p = squeeze(out.trackingErrpr.Data)';
norm_e_p = vecnorm(e_p,2,2);
plot(t_flight, norm_e_p)
title("Norm of the tracking error $\|p - p_s\|_2$", 'Interpreter','latex');
xlabel("t in [s]");
ylabel("$\|p - p_s\|_2$", 'Interpreter','latex');
zielOrdner = 'C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\data';
dateiname = 'norm_e_p.mat';

save(fullfile(zielOrdner, dateiname), 'norm_e_p');