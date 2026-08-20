function test_flatness_blocks()
%test_flatness_blocks  Selbsttest der Codegen-Bloecke flatness_ctrl + flatness_khat
%   gegen die Strecke (Motor-PT1, Gamma-Mixer). Testet zusaetzlich den
%   Quaternion-Round-Trip (Plant-R -> q -> Block-R).
clear flatness_ctrl flatness_khat
here = fileparts(mfilename('fullpath'));
sim_root = fileparts(fileparts(here));
addpath(fullfile(sim_root,'scripts','init'));
addpath(fullfile(sim_root,'scripts','functions'));
addpath(here);

rng(42);
g = 9.81;
[~, quadcop] = evalc('init_quadcop()'); % evalc schluckt die Init-Ausgaben
fctrl = init_flatness(quadcop);
m=quadcop.m; J=quadcop.J; Gam=quadcop.Gamma; Gaminv=quadcop.Gamma_inv;
wmin=quadcop.rotors_min; wmax=quadcop.rotors_max; tau_m=0.030;
% Der Schubmangel steckt bereits in quadcop.Gamma (Faktor quadcop.k_thrust).
thrust_scale=1.0;
k_soll = thrust_scale * quadcop.k_thrust; % Wert, auf den k_hat laufen muss
% Die Testtrajektorie liegt durchgehend ueber 0.8 m -> Hoehenrampe voll offen.
w_adapt = 1.0;

% Trajektorie (restpoly, konstanter Yaw)
T=8; dt=0.002; tdisc=0:dt:T; Nt=numel(tdisc);
p0=[0;0;0.8]; pT=[1;1;1.5]; Dp=pT-p0;
P=zeros(3,Nt);Vt=zeros(3,Nt);A=zeros(3,Nt);Jt=zeros(3,Nt);St=zeros(3,Nt);
for k=1:Nt
  [s0,s1,s2,s3,s4]=rp(tdisc(k)/T);
  P(:,k)=p0+Dp*s0; Vt(:,k)=Dp*s1/T; A(:,k)=Dp*s2/T^2; Jt(:,k)=Dp*s3/T^3; St(:,k)=Dp*s4/T^4;
end
yawref=[0;0;0];

% Anfangszustand (14 cm Offset) + Motoren auf Hover
wsq_h=Gaminv*[m*g;0;0;0]; w_h=sqrt(max(wsq_h,0));
states=zeros(22,Nt);
states(:,1)=[0.1;-0.1;0.8; 0;0;0; reshape(eye(3),9,1); 0;0;0; w_h];

k_hat=fctrl.k_hat0; F_prev=m*g; sigma_v=0.01; Tm=0.01; t_moc_prev=-Tm;
khist=zeros(1,Nt);

for i=1:Nt-1
  p=states(1:3,i); v=states(4:6,i);
  R=[states(7:9,i) states(10:12,i) states(13:15,i)]; w=states(16:18,i);
  q = dcm2quat_local(R.'); % Plant-R -> Quaternion

  % --- Schub-Skalenschaetzer @100 Hz ---
  if tdisc(i)-t_moc_prev >= Tm-1e-9
    v_obs = v + sigma_v*randn(3,1); % Luenberger-Ausgang emuliert
    k_hat = flatness_khat(v_obs, q, F_prev, m, g, fctrl.gamma_khat, fctrl.tau_lp, Tm, ...
                          fctrl.k_hat0, w_adapt);
    t_moc_prev=tdisc(i);
  end
  khist(i)=k_hat;

  % --- flachheitsbasierter Regler @500 Hz ---
  [F,tau] = flatness_ctrl(p,v,q,w, P(:,i),Vt(:,i),A(:,i),Jt(:,i),St(:,i), yawref, ...
                          k_hat, m, g, J, fctrl.coefPos, fctrl.coefPhi, dt, false, w_adapt);
  F_prev=F;

  % --- Mixer + Strecke (Motor-PT1) ---
  wsq=Gaminv*[F;tau]; wc=sqrt(max(wsq,0)); wc=min(max(wc,wmin),wmax);
  par=struct('g',g,'D',diag([0 0 0]),'I',J,'m',m,'thrust_scale',thrust_scale,'Gam',Gam,'tau_m',tau_m);
  [~,y]=ode45(@(t,y) plnt(t,y,wc,par),[tdisc(i) tdisc(i+1)],states(:,i));
  states(:,i+1)=y(end,:)';
end
khist(Nt)=k_hat;

perr=states(1:3,:)-P; enrm=sqrt(sum(perr.^2,1));
fprintf('=== Selbsttest flatness_ctrl + flatness_khat (ts=%.2f, Lag 30 ms) ===\n', k_soll);
fprintf('Trackingfehler: max %.4f m   final %.4f m\n', max(enrm), enrm(end));
fprintf('k_hat final:    %.4f   (Ziel %.2f)\n', khist(end), k_soll);
if enrm(end) < 0.02 && abs(khist(end)-k_soll) < 0.03
  fprintf('ERGEBNIS: BESTANDEN (reproduziert validierte Sim)\n');
else
  fprintf('ERGEBNIS: ABWEICHUNG -> pruefen\n');
end
end

function dydt=plnt(~,y,wc,p)
  v=y(4:6); R=[y(7:9) y(10:12) y(13:15)]; w=y(16:18); wm=y(19:22);
  hw=[0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
  wr=p.Gam*(wm.^2); a=p.thrust_scale*wr(1)/p.m; ta=wr(2:4);
  dydt=zeros(22,1); dydt(1:3)=v; dydt(4:6)=-p.g*[0;0;1]+a*R(:,3)-R*p.D*R'*v;
  dR=R*hw; dydt(7:15)=[dR(:,1);dR(:,2);dR(:,3)];
  dydt(16:18)=p.I\(ta-cross(w,p.I*w)); dydt(19:22)=(wc-wm)/p.tau_m;
end

function [s0,s1,s2,s3,s4]=rp(t)
  if t<0,t=0;elseif t>1,t=1;end
  t2=t*t;t3=t2*t;t4=t3*t;t5=t4*t;t6=t5*t;t7=t6*t;t8=t7*t;t9=t8*t;
  s0=126*t5-420*t6+540*t7-315*t8+70*t9; s1=630*t4-2520*t5+3780*t6-2520*t7+630*t8;
  s2=2520*t3-12600*t4+22680*t5-17640*t6+5040*t7; s3=7560*t2-50400*t3+113400*t4-105840*t5+35280*t6;
  s4=15120*t-151200*t2+453600*t3-529200*t4+211680*t5;
end
