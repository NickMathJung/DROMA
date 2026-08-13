function [x_ref, v_ref, a_ref, yaw_ref, Omega_ref, tau_ref, q_ref, F_ref, j_ref, s_ref] = traj_gen(t, traj, quadcop, d)
%#codegen
% traj_gen  Minimum-Snap-Trajektorie siehe Paper Mellinger und Kumar (2011).
%
%   Liefert die Trajektorien für die flachen Ausgänge beim Arbeitspunktwechsel
%   und daraus die Vorsteuerung.
%
%   2-DOF-Prinzip:
%     x_ref, v_ref, a_ref, yaw_ref  -> Positionsfolgeregler (äußere Kaskade)
%     q_ref, Omega_ref, tau_ff      -> Bus_Cmd   (Vorsteuerung)
%     F_ref                         -> nur Debug/Verifikation 
%
%   Eingaenge:
%     t    : Simulationszeit [s] 
%     traj : struct, N >= 2
%                  .P      3 x N      Wegpunkte
%                  .yaw    1 x (N-1)  Yaw je Segment
%                  .Tseg   1 x (N-1)  Bewegungsdauer je Segment
%                  .Tdwell 1 x N      Rastdauer je Wegpunkt
%     quadcop  : struct  .m  .g  .J
%     d    : optional Drohnenindex (Schwarm: traj-Felder mit gestapelter
%            3. Dim bzw. Zeile je Drohne; Einzelstrukturen laufen mit d=1)
%
%   Ausgänge:
%     x_ref,v_ref,a_ref 3x1 Soll-Pos/Geschw/Beschl 
%     yaw_ref     3x1 [yaw; dyaw; ddyaw]  
%     Omega_ref   3x1 Vorsteuer-Drehrate (Body) 
%     tau_ff      3x1 Vorsteuer-Moment (Body) 
%     q_ref       4x1 nominelles Soll-Quaternion
%     F_ref       1x1 nomineller Schubbetrag -- Debug
%

    g_grav = [0; 0; quadcop.g];
    if nargin < 4
        d = 1; % Einzeldrohnen-Aufrufe (Sim/SITL/Tests)
    end
    trj = traj_slice(traj, d);
    N = size(trj.P, 2);

    % ------ Tabelle für Schwarm: Referenz aus vorberechneter Tabelle -----
    % traj.tab_* kommt aus init_trajectory_swarm (swarm_ref.mat).
    % Ohne/mit leerem tab_p fällt der Zweig zur Codegen-Zeit weg
    if isfield(trj, 'tab_p') && ~isempty(trj.tab_p)
        [x_ref, v_ref, a_ref, j_ref] = tab_lookup(t, trj);
        s_ref = zeros(3,1);   % Snap nicht tabelliert; nur Vorsteuer-Feinheit
        yaw_s = 0;            % Schwarm fliegt yaw = 0
        yaw_ref = [yaw_s; 0; 0];
        [Omega_ref, tau_ref, q_ref, F_ref] = ...
            flat_ff(a_ref, j_ref, s_ref, yaw_s, g_grav, quadcop);
        return;
    end

    % --- Phase bestimmen ---
    % Phasenfolge: dwell(1), move(1), dwell(2), move(2), ..., dwell(N)
    mode    = int8(2); % 0=Rast, 1=Bewegung, 2=End-Halt 
    sel_wp  = N; % aktiver Wegpunkt (Rast/Halt)
    sel_seg = 1; % aktives Segment (Bewegung)
    tloc    = 0.0; % lokale Zeit in der Phase

    if t < 0
        mode = int8(0); 
        sel_wp = 1; 
        tloc = 0.0; % vor Start: WP1 halten
    else
        acc = 0.0;
        for i = 1:N
            Dd = trj.Tdwell(i); % Rast an WP i
            if (t >= acc) && (t < acc + Dd)
                mode = int8(0); 
                sel_wp = i; 
                tloc = t - acc; 
                break;
            end
            acc = acc + Dd;
            if i < N
                Tm = trj.Tseg(i); % Bewegung WP i -> i+1
                if (t >= acc) && (t < acc + Tm)
                    mode = int8(1); 
                    sel_seg = i; 
                    tloc = t - acc; 
                    break;
                end
                acc = acc + Tm;
            end
        end
        % kein break -> mode bleibt 2 (End-Halt an WP N)
    end

    % --- Trajektorie erzeugen ---
    if mode == int8(1) % fliegt Trajektoriensegment ab
        k  = sel_seg;
        T  = trj.Tseg(k);
        tau = tloc / T;
        [s0,s1,s2,s3,s4] = restpoly(tau);
        D   = trj.P(:,k+1) - trj.P(:,k); % Differenz zwischen den Wegpunkten
        x_ref = trj.P(:,k) + D*s0;
        v_ref = D*s1 / T;
        a_ref = D*s2 / T^2;
        j_ref = D*s3 / T^3;
        s_ref = D*s4 / T^4;
        yaw_s = trj.yaw(k);
    elseif mode == int8(0) % Rastpunkt
        x_ref = trj.P(:,sel_wp);
        v_ref = zeros(3,1);  
        a_ref = zeros(3,1);
        j_ref = zeros(3,1);
        s_ref = zeros(3,1);
        if sel_wp < N
            yaw_s = trj.yaw(sel_wp);    % Yaw des kommenden Segments
        else
            yaw_s = trj.yaw(N-1);
        end
    else % End-Halt
        x_ref = trj.P(:,N);
        v_ref = zeros(3,1);
        a_ref = zeros(3,1);
        j_ref = zeros(3,1);
        s_ref = zeros(3,1);
        yaw_s = trj.yaw(N-1);
    end

    % Yaw-Ausgang als 3x1 [yaw; dyaw; ddyaw]; Segment-Yaw -> Ableitungen 0.
    % (Fuer flatness_ctrl.yawref; die Kaskade nutzt in pos_ctrl nur yaw_ref(1).)
    yaw_ref = [yaw_s; 0; 0];

    [Omega_ref, tau_ref, q_ref, F_ref] = ...
        flat_ff(a_ref, j_ref, s_ref, yaw_s, g_grav, quadcop);
end

% --- Vorsteuerung aus flachen Ausgängen (gemeinsam fuer beide Modi) ------
function [Omega_ref, tau_ref, q_ref, F_ref] = flat_ff(a_ref, j_ref, s_ref, yaw_s, g_grav, quadcop)
    % --- Flache Ausgänge und ihre Ableitungen in die Sollzustände umrechnen ---
    % Schubachse:  F*z_B = m*alpha,  alpha = a_ref - g_grav
    alpha = a_ref + g_grav;
    alphad = j_ref;
    alphadd = s_ref;

    n   = norm(alpha); % > 0 (nahe Hover ~ g)
    zB  = alpha / n;
    nd  = (alpha.'*alphad) / n;
    ndd = ((alphad.'*alphad) + (alpha.'*alphadd))/n - nd^2/n;
    zBd  = alphad/n - alpha*nd/n^2;
    zBdd = alphadd/n - 2*alphad*nd/n^2 - alpha*ndd/n^2 + 2*alpha*nd^2/n^3;

    % Heading (konstanter Yaw je Segment -> xC konstant, Ableitungen 0)
    xC = [cos(yaw_s); sin(yaw_s); 0];

    c   = cross(zB,   xC);
    cd  = cross(zBd,  xC);
    cdd = cross(zBdd, xC);
    nc   = norm(c); % Schutz: |zB x xC| ~ 1 nahe Hover
    if nc < 1e-6
        nc = 1e-6;
    end
    ncd  = (c.'*cd) / nc;
    ncdd = ((cd.'*cd) + (c.'*cdd))/nc - ncd^2/nc;
    yB   = c/nc;
    yBd  = cd/nc - c*ncd/nc^2;
    yBdd = cdd/nc - 2*cd*ncd/nc^2 - c*ncdd/nc^2 + 2*c*ncd^2/nc^3;

    xB   = cross(yB,   zB);
    xBd  = cross(yBd,  zB) + cross(yB,  zBd);
    xBdd = cross(yBdd, zB) + 2*cross(yBd, zBd) + cross(yB, zBdd);

    Rd   = [xB,   yB,   zB]; % = R_{n<-b}, Body -> Inertial
    Rdd  = [xBd,  yBd,  zBd]; % d/dt R_d
    Rddd = [xBdd, yBdd, zBdd]; % d2/dt2 R_d

    % [Omega]x = Rd' * Rdd <-> Rdd = Rd * [Omega]x
    Om_hat   = Rd.' * Rdd;
    Omega_ref = vee(0.5*(Om_hat - Om_hat.')); % schiefsymmetrischen Anteil nehmen
    % [Omegadot]x = Rd'*Rddd - [Omega]x^2
    Omd_hat  = Rd.' * Rddd - Om_hat*Om_hat;
    Omega_dot = vee(0.5*(Omd_hat - Omd_hat.'));

    tau_ref    = quadcop.J*Omega_dot + cross(Omega_ref, quadcop.J*Omega_ref);
    F_ref = quadcop.m * n;
    q_ref  = dcm2quat_local(Rd.'); % Rd' = R_{b<-n}
end

% --- Lokale Helfer ---
function trj = traj_slice(traj, d)
% Scheibe der Drohne d: P/tab_* ueber die 3. Dim, yaw/Tseg/Tdwell zeilenweise.
% Einzeldrohnen-Strukturen (2-D/einzeilig) laufen mit d=1 unveraendert durch.
    trj.P      = traj.P(:,:,d);
    trj.yaw    = traj.yaw(d,:);
    trj.Tseg   = traj.Tseg(d,:);
    trj.Tdwell = traj.Tdwell(d,:);
    if isfield(traj, 'tab_p')
        trj.tab_Ts = traj.tab_Ts;
        trj.tab_p  = traj.tab_p(:,:,d);
        trj.tab_v  = traj.tab_v(:,:,d);
        trj.tab_a  = traj.tab_a(:,:,d);
        trj.tab_j  = traj.tab_j(:,:,d);
    end
end

function [p, v, a, j] = tab_lookup(t, traj)
% Lineare Interpolation auf dem uniformen tab_Ts-Raster; nach Tabellenende
% Position halten (v/a/j -> 0), vor t=0 den Startpunkt.
    Ts = traj.tab_Ts;
    Tn = size(traj.tab_p, 1);
    tend = (Tn - 1) * Ts;
    tt = min(max(t, 0), tend);
    k = min(floor(tt / Ts) + 1, Tn - 1);
    f = (tt - (k-1)*Ts) / Ts;
    p = ((1-f)*traj.tab_p(k,:) + f*traj.tab_p(k+1,:)).';
    v = ((1-f)*traj.tab_v(k,:) + f*traj.tab_v(k+1,:)).';
    a = ((1-f)*traj.tab_a(k,:) + f*traj.tab_a(k+1,:)).';
    j = ((1-f)*traj.tab_j(k,:) + f*traj.tab_j(k+1,:)).';
    if t <= 0 || t >= tend
        v = zeros(3,1); a = zeros(3,1); j = zeros(3,1);
    end
end

function [s0,s1,s2,s3,s4] = restpoly(tau)
% Minimum-Crackle Arbeitspunktwechsel 0->1 über tau in [0,1].
% s und die ersten vier Ableitungen (v,a,j,s) sind an beiden Enden null,
% also Grad 9. Der Snap ist an den Enden stetig (==0), damit tau_ref an den
% Wegpunktuebergaengen nicht springt.
    if tau < 0
        tau = 0;
    elseif tau > 1
        tau = 1;
    end
    t2 = tau*tau;
    t3 = t2*tau;
    t4 = t3*tau;
    t5 = t4*tau;
    t6 = t5*tau;
    t7 = t6*tau;
    t8 = t7*tau;
    t9 = t8*tau;
    s0 =   126*t5 -   420*t6 +   540*t7 -   315*t8 +  70*t9;
    s1 =   630*t4 -  2520*t5 +  3780*t6 -  2520*t7 + 630*t8;   % ds/dtau
    s2 =  2520*t3 - 12600*t4 + 22680*t5 - 17640*t6 + 5040*t7;  % d2s/dtau2
    s3 =  7560*t2 - 50400*t3 + 113400*t4 - 105840*t5 + 35280*t6; % d3s/dtau3
    s4 = 15120*tau - 151200*t2 + 453600*t3 - 529200*t4 + 211680*t5; % d4s/dtau4
end

function v = vee(S)
    v = [S(3,2); S(1,3); S(2,1)];
end

function q = dcm2quat_local(R)
% Shepperd-Algorithmus, Skalar zuerst, R = Inertial->Koerper (R_{b<-n})
    r11=R(1,1); r22=R(2,2); r33=R(3,3);
    tr = r11+r22+r33;
    if tr > 0
        S = 2*sqrt(1+tr);
        q0 = 0.25*S;
        q1 = (R(2,3)-R(3,2))/S;
        q2 = (R(3,1)-R(1,3))/S;
        q3 = (R(1,2)-R(2,1))/S;
    elseif (r11 > r22) && (r11 > r33)
        S = 2*sqrt(1+r11-r22-r33);
        q0 = (R(2,3)-R(3,2))/S;
        q1 = 0.25*S;
        q2 = (R(1,2)+R(2,1))/S;
        q3 = (R(3,1)+R(1,3))/S;
    elseif r22 > r33
        S = 2*sqrt(1+r22-r11-r33);
        q0 = (R(3,1)-R(1,3))/S;
        q1 = (R(1,2)+R(2,1))/S;
        q2 = 0.25*S;
        q3 = (R(2,3)+R(3,2))/S;
    else
        S = 2*sqrt(1+r33-r11-r22);
        q0 = (R(1,2)-R(2,1))/S;
        q1 = (R(3,1)+R(1,3))/S;
        q2 = (R(2,3)+R(3,2))/S;
        q3 = 0.25*S;
    end
    q = [q0; q1; q2; q3];
    if q0 < 0, q = -q; end
    q = q / norm(q);
end
