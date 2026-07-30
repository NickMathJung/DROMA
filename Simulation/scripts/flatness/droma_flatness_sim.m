function droma_flatness_sim(I_ctrl_scale, thrust_scale, gamma_khat, tau_m, meas_src)
% droma_flatness_sim  Phase-0/1: flachheitsbasierter Folgeregler (exakte
%   Linearisierung) + Schub-Skalenschaetzer mit DROMA-Parametern, standalone,
%   z-up, toolbox-frei. Jetzt MIT Motor-PT1 + Gamma-Mixer + Saettigung.
%
%   Regelkern VERBATIM aus mainFlatnessTracking.m /
%   exactFeedbackLinearizedDroneWithNetworkedController.m (z-up, Schub +R(:,3),
%   F=zeta1 spez. Schub). Aktuatorpfad: Regler-Wrench [F_N;tau] -> Gamma_inv ->
%   w_i^2_cmd -> sqrt/Saettigung -> Motor-PT1 (tau_m) -> Gamma -> echte [F;tau].
%
%   I_ctrl_scale : Faktor auf REGLER-Traegheit (Strecke behaelt wahres I).
%   thrust_scale : aerodyn. Schub-Download auf die gelieferte Schubkraft (1=exakt).
%   gamma_khat   : Adaptionsgain Schub-Skalenschaetzer (0=aus). F_cmd=zeta1/k_hat,
%                  k_hat_dot=gamma*(s_mess-zeta1), s_mess=zB'*(a_ist+g)=IMU-z.
%   tau_m        : Motor-Zeitkonstante [s] (DROMA: 0.030). <=0 -> 0.001 (quasi ideal).
if nargin < 1, I_ctrl_scale = 1.0; end
if nargin < 2, thrust_scale = 1.0; end
if nargin < 3, gamma_khat  = 0.0; end
if nargin < 4, tau_m       = 0.030; end
if nargin < 5, meas_src    = 'truth'; end   % 'truth' | 'imu' | 'mocap'
if tau_m <= 0, tau_m = 0.001; end

here = fileparts(mfilename('fullpath'));
sim_root = fileparts(fileparts(here));   % scripts/flatness/ -> scripts -> Simulation (portabel)
addpath(fullfile(sim_root,'scripts','init'));
addpath(fullfile(sim_root,'scripts','functions'));

g = 9.81;
quadcop = evalc_init(sim_root);
m    = quadcop.m;
I    = quadcop.J;                    % WAHRE Traegheit (Strecke)
I_c  = I_ctrl_scale * I;             % Regler-Traegheit (evtl. verstimmt)
D    = diag([0 0 0]);
Gam  = quadcop.Gamma;                % [F_N; tau] = Gamma * w_i^2
Gaminv = quadcop.Gamma_inv;
wmin = quadcop.rotors_min; wmax = quadcop.rotors_max;
params = struct('g',g,'I',I,'D',D,'m',m,'thrust_scale',thrust_scale,'Gam',Gam,'tau_m',tau_m);
w_rotor_max = 28597 * 2*pi/60;       % ~2994 rad/s aero-Envelope (Reporting)

%% ---- Trajektorie (flache Ausgaenge), restpoly, konstanter Yaw ----
T = 8; dt = 0.002; tdisc = 0:dt:T; Nt = numel(tdisc);
p0 = [0;0;0.8]; pT = [1.0;1.0;1.5];
Dp = pT - p0;
P=zeros(3,Nt);V=zeros(3,Nt);A=zeros(3,Nt);J=zeros(3,Nt);S=zeros(3,Nt);
for k=1:Nt
  [s0,s1,s2,s3,s4]=restpoly(tdisc(k)/T);
  P(:,k)=p0+Dp*s0; V(:,k)=Dp*s1/T; A(:,k)=Dp*s2/T^2; J(:,k)=Dp*s3/T^3; S(:,k)=Dp*s4/T^4;
end
flat.p=P; flat.v=V; flat.a=A; flat.j=J; flat.s=S;
flat.phi=zeros(1,Nt); flat.dphi=zeros(1,Nt); flat.ddphi=zeros(1,Nt);

%% ---- Brunovsky-Trackinggains (aus Referenz) ----
eigenvalPos = [ [2 2 2 2]; [1.5 2 2 2]; 2.0*[2 2 2 2] ];
coefPos = zeros(3,5);
for jr=1:3
  cp=[1 eigenvalPos(jr,1)];
  for ii=1:3, cp=conv(cp,[1 eigenvalPos(jr,ii+1)]); end
  coefPos(jr,:)=cp;
end
eigenvalPhi=[1 2];
coefPhi=conv([1 eigenvalPhi(2)],[1 eigenvalPhi(1)]);

%% ---- Anfangszustand: 14 cm Offset + Motoren auf Hover-Drehzahl ----
p_init=[0.1; -0.1; 0.8];
wsq_hover = Gaminv*[m*g;0;0;0];
w_hover   = sqrt(max(wsq_hover,0));            % 4x1
states=zeros(22,Nt);
states(:,1)=[p_init; 0;0;0; reshape(eye(3),9,1); 0;0;0; w_hover];
zeta1=g; zeta2=0; eZ=[0;0;1]; eY=[0;1;0];
k_hat=1.0;
Fhist=zeros(1,Nt); tauhist=zeros(3,Nt); zeta1hist=zeros(1,Nt); khist=zeros(1,Nt);
wcmd_max=0; sat_hit=false;

% --- Messquellen-Emulation fuer den k_hat-Schaetzer ---
rng(42);
imu_fvib=70; imu_avib=3.0; imu_sigma=0.5; imu_bias=0.3;   % [m/s^2] Vibration/Rauschen/BIAS
Tm=0.01; moc_sigma_p=2e-4; tau_lp=0.10;                   % Mocap 100 Hz, 0.2 mm Rauschen, 1.6-Hz-TP
p_moc_prev=states(1:3,1); v_moc_prev=[0;0;0]; a_moc_lp=[0;0;0];
t_moc_prev=-Tm; s_meas_hold=g;
% velocity-innovation-Schaetzer (HW-Form: Luenberger-v vs Modellpraediktion)
v_pred=[0;0;0]; L_obs=0.5; sigma_v=0.01;                  % Luenberger-v-Rauschen ~1 cm/s

for i=1:Nt-1
  p=states(1:3,i); v=states(4:6,i);
  R=[states(7:9,i) states(10:12,i) states(13:15,i)]; w=states(16:18,i);
  w_mot=states(19:22,i);
  tw=[0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];

  % --- gelieferter spez. Schub (Ground Truth, verzoegert durch Motor-PT1) ---
  wrench_act = Gam*(w_mot.^2);
  s_truth = thrust_scale*wrench_act(1)/m - R(:,3)'*(R*D*R'*v);
  % --- Messung des Schaetzers je nach Quelle ---
  switch meas_src
    case 'imu'    % Beschleunigungsmesser: Ground Truth + Vibration + Rauschen + BIAS
      s_meas = s_truth + imu_avib*sin(2*pi*imu_fvib*tdisc(i)) + imu_sigma*randn + imu_bias;
    case 'mocap'  % 100-Hz-Pos -> doppelt differenziert -> TP -> auf zB projiziert (bias-frei)
      if tdisc(i) - t_moc_prev >= Tm - 1e-9
        p_moc = p + moc_sigma_p*randn(3,1);
        v_moc = (p_moc - p_moc_prev)/Tm;
        a_moc = (v_moc - v_moc_prev)/Tm;
        alp = Tm/(Tm+tau_lp); a_moc_lp = a_moc_lp + alp*(a_moc - a_moc_lp);
        s_meas_hold = R(:,3)'*(a_moc_lp + g*eZ);
        p_moc_prev=p_moc; v_moc_prev=v_moc; t_moc_prev=tdisc(i);
      end
      s_meas = s_meas_hold;
    case 'mocap_av'  % Beschl. aus EINMALIGER Diff. der Beobachter-v (bias-frei, HW-Form)
      if tdisc(i) - t_moc_prev >= Tm - 1e-9
        v_obs = v + sigma_v*randn(3,1);                   % Luenberger-Ausgang
        a_obs = (v_obs - v_moc_prev)/Tm;                  % einmalig differenziert
        alp = Tm/(Tm+tau_lp); a_moc_lp = a_moc_lp + alp*(a_obs - a_moc_lp);
        s_meas_hold = R(:,3)'*(a_moc_lp + g*eZ);
        v_moc_prev = v_obs; t_moc_prev = tdisc(i);
      end
      s_meas = s_meas_hold;
    case 'mocap_vi'  % velocity-innovation: Luenberger-v (bias-frei) vs Modellpraediktion
      v_pred = v_pred + (-g*eZ + zeta1*R(:,3))*dt;        % Praediktion mit Modellbeschl.
      if tdisc(i) - t_moc_prev >= Tm - 1e-9
        v_obs = v + sigma_v*randn(3,1);                   % Luenberger-Ausgang (mocap-basiert)
        e_v = v_obs - v_pred;
        k_hat = k_hat + gamma_khat * (R(:,3)'*e_v);       % Innovation entlang zB treibt k_hat
        v_pred = v_pred + L_obs*e_v;                      % Beobachter-Korrektur
        t_moc_prev = tdisc(i);
      end
      s_meas = zeta1;   % neutralisiert die generische Update-Zeile unten
    otherwise     % 'truth'
      s_meas = s_truth;
  end

  % --- Flachheits-Regelgesetz (verbatim) ---
  a = -g*eZ + zeta1*R(:,3);
  jj = zeta2*R(:,3) + zeta1*R*tw*eZ;
  projC=R(:,2)'*eZ; yc=R(:,2)-projC*eZ; yC=yc/norm(yc);
  phi_=acos(eY'*yC); xC=[cos(phi_);sin(phi_);0];
  txB=cross(yC,R(:,3));
  dphi_=(norm(txB)*w(3)-w(2)*yC'*R(:,3))/(xC'*R(:,1));

  u_123 = flat.s(:,i) - diag(coefPos(:,2))*(jj-flat.j(:,i)) - diag(coefPos(:,3))*(a-flat.a(:,i)) ...
          - diag(coefPos(:,4))*(v-flat.v(:,i)) - diag(coefPos(:,5))*(p-flat.p(:,i));
  u_4 = flat.ddphi(i) - coefPhi(2)*(dphi_-flat.dphi(i)) - coefPhi(3)*(phi_-flat.phi(i));

  dwx=(-R(:,2)'*u_123 - 2*zeta2*w(1) + zeta1*w(2)*w(3))/zeta1;
  dwy=( R(:,1)'*u_123 - 2*zeta2*w(2) + zeta1*w(1)*w(3))/zeta1;
  dwz=( zeta1*(u_4*xC'*R(:,1) + 2*dphi_*w(3)*xC'*R(:,2) - 2*dphi_*w(2)*xC'*R(:,3) ...
        - w(1)*w(2)*yC'*R(:,2) - w(1)*w(3)*yC'*R(:,3)) ...
        + yC'*R(:,3)*(R(:,1)'*u_123 - 2*zeta2*w(2) - zeta1*w(1)*w(3)))/zeta1 * norm(cross(yC,R(:,3)));
  dw=[dwx;dwy;dwz];
  tau = I_c*dw + cross(w,I_c*w);

  % --- Schub-Skalenschaetzer ---
  F_cmd = zeta1 / max(k_hat,1e-3);              % kommandierter spez. Schub
  k_hat = k_hat + gamma_khat*(s_meas - zeta1)*dt;
  k_hat = min(max(k_hat,0.5),1.5);
  zeta1hist(i)=zeta1; khist(i)=k_hat;

  % --- Allokation ueber echten Mixer -> w_i^2_cmd -> Saettigung ---
  wsq_cmd = Gaminv*[m*F_cmd; tau];
  w_cmd   = sqrt(max(wsq_cmd,0));
  w_cmd   = min(max(w_cmd,wmin),wmax);
  if any(wsq_cmd<0) || any(w_cmd>=wmax), sat_hit=true; end
  wcmd_max = max(wcmd_max, max(w_cmd));
  Fhist(i)=thrust_scale*wrench_act(1); tauhist(:,i)=wrench_act(2:4); % geloggt: ECHT geliefert

  % --- zeta integrieren ---
  z2o=zeta2;
  dz2 = R(:,3)'*u_123 - 2*zeta2*R(:,3)'*R*tw*eZ + zeta1*R(:,3)'*R*(tw^2 + dw)*eZ;
  zeta2 = zeta2 + dz2*dt;
  zeta1 = zeta1 + z2o*dt;

  % --- Strecke mit Motor-PT1 integrieren (w_cmd ueber Schritt gehalten) ---
  [~,y]=ode45(@(t,y) plant(t,y,w_cmd,params),[tdisc(i) tdisc(i+1)],states(:,i));
  states(:,i+1)=y(end,:)';
end
Fhist(Nt)=Fhist(Nt-1); tauhist(:,Nt)=tauhist(:,Nt-1); zeta1hist(Nt)=zeta1; khist(Nt)=k_hat;

%% ---- Auswertung ----
perr = states(1:3,:) - flat.p;
enrm = sqrt(sum(perr.^2,1));

fprintf('=== DROMA Flatness  (I_scale=%.2f, thrust_scale=%.2f, gamma=%.2f, tau_m=%.3fs, mess=%s) ===\n', ...
        I_ctrl_scale, thrust_scale, gamma_khat, tau_m, meas_src);
fprintf('Trackingfehler-Norm:   max %.4f m   final %.4f m   (Start-Offset 0.141 m)\n', max(enrm), enrm(end));
fprintf('Schub F (geliefert):   min %.3f N   max %.3f N   (Hover %.3f N)\n', min(Fhist), max(Fhist), m*g);
fprintf('zeta1 (spez. Schub):   min %.3f     (>0 = kein Schubumkehr)\n', min(zeta1hist));
fprintf('|tau| je Achse:        max [%.4f %.4f %.4f] Nm\n', max(abs(tauhist(1,:))), max(abs(tauhist(2,:))), max(abs(tauhist(3,:))));
fprintf('groesste w_cmd:        %.1f rad/s  von %.1f Envelope (%.0f%%)  Saettigung: %s\n', ...
        wcmd_max, w_rotor_max, 100*wcmd_max/w_rotor_max, ternary(sat_hit,'JA','nein'));
fprintf('Schub-Schaetzer k_hat: final %.4f  (Ziel thrust_scale=%.2f)\n', khist(end), thrust_scale);

f=figure('Visible','off','Position',[100 100 1200 640]);
subplot(2,3,1); plot(tdisc,enrm,'LineWidth',1.2); grid on; ylabel('|p-p_{soll}| [m]'); xlabel('t [s]'); title('Trackingfehler-Norm');
subplot(2,3,2); plot(tdisc,states(1:3,:)','LineWidth',1); hold on; set(gca,'ColorOrderIndex',1); plot(tdisc,flat.p','--'); grid on; ylabel('p [m]'); xlabel('t [s]'); title('Pos (—ist / --soll)'); legend('x','y','z');
subplot(2,3,3); plot(tdisc,khist,'LineWidth',1.4); hold on; yline(thrust_scale,'r--'); grid on; ylabel('k\_hat'); xlabel('t [s]'); title('Schub-Skalenschaetzer (rot=Ziel)');
subplot(2,3,4); plot(tdisc,Fhist,'LineWidth',1.2); yline(m*g,'r--'); grid on; ylabel('F [N]'); xlabel('t [s]'); title('gelief. Schub (rot=Hover)');
subplot(2,3,5); plot(tdisc,tauhist','LineWidth',1); grid on; ylabel('\tau [Nm]'); xlabel('t [s]'); title('gelief. Stellmoment'); legend('\tau_x','\tau_y','\tau_z');
subplot(2,3,6); plot(tdisc,zeta1hist,'LineWidth',1.2); grid on; ylabel('\zeta_1 [m/s^2]'); xlabel('t [s]'); title('interner Schubzustand \zeta_1');
outpng = fullfile(here, sprintf('droma_flatness_ts%.2f_g%.2f_tm%.0fms_%s.png', thrust_scale, gamma_khat, 1000*tau_m, meas_src));
saveas(f, outpng); close(f);
fprintf('Plot: %s\n\n', outpng);
end

%% ================= Hilfsfunktionen =================
function dydt = plant(~, y, w_cmd, params)
  v=y(4:6); R=[y(7:9) y(10:12) y(13:15)]; w=y(16:18); w_mot=y(19:22);
  hw=[0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
  g=params.g; D=params.D; I=params.I; ts=params.thrust_scale; Gam=params.Gam; tau_m=params.tau_m;
  wrench = Gam*(w_mot.^2);            % [F_N; tau] aus echten Motordrehzahlen
  a_spec = ts*wrench(1)/params.m;     % spez. Schub inkl. aerodyn. Download
  tau_act = wrench(2:4);
  dydt=zeros(22,1);
  dydt(1:3)=v;
  dydt(4:6)=-g*[0;0;1] + a_spec*R(:,3) - R*D*R'*v;
  dR=R*hw; dydt(7:15)=[dR(:,1);dR(:,2);dR(:,3)];
  dydt(16:18)=I\(tau_act - cross(w,I*w));
  dydt(19:22)=(w_cmd - w_mot)/tau_m;  % Motor-PT1
end

function [s0,s1,s2,s3,s4]=restpoly(tau)
  if tau<0, tau=0; elseif tau>1, tau=1; end
  t2=tau*tau;t3=t2*tau;t4=t3*tau;t5=t4*tau;t6=t5*tau;t7=t6*tau;t8=t7*tau;t9=t8*tau;
  s0=126*t5-420*t6+540*t7-315*t8+70*t9;
  s1=630*t4-2520*t5+3780*t6-2520*t7+630*t8;
  s2=2520*t3-12600*t4+22680*t5-17640*t6+5040*t7;
  s3=7560*t2-50400*t3+113400*t4-105840*t5+35280*t6;
  s4=15120*tau-151200*t2+453600*t3-529200*t4+211680*t5;
end

function quadcop = evalc_init(~)
  evalc('quadcop = init_quadcop();');
end

function out = ternary(c,a,b)
  if c, out=a; else, out=b; end
end
