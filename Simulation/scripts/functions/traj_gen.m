function [x_ref, v_ref, a_ref, yaw_ref, Omega_ref, tau_ref, q_ref, F_ref] = traj_gen(t, traj, quadcop)
%#codegen
% traj_gen  Minimum-Snap Punkt-zu-Punkt-Trajektorie.
%
%   Liefert die Trajektorien für die flachen Ausgänge beim Arbeitspunktwechsel
%   und daraus die Vorsteuerung.
%
%   Verdrahtung (2-DOF):
%     x_ref, v_ref, a_ref, yaw_ref  -> Positionsfolgeregler (äußere Kaskade)
%     q_ref, Omega_ref, tau_ff      -> Bus_Cmd   (reine Vorsteuerung)
%     F_ref   -> nur Debug/Verifikation (optional, unverbunden ok)
%
%   Eingaenge:
%     t    : Simulationszeit [s]  (z.B. Clock-Block)
%     traj : struct, N >= 2
%                  .P      3 x N      Wegpunkte
%                  .yaw    1 x (N-1)  Yaw je Segment (konstant je Segment)
%                  .Tseg   1 x (N-1)  Bewegungsdauer je Segment
%                  .Tdwell 1 x N      Rastdauer je Wegpunkt
%     quadcop  : struct  .m  .g  .J
%
%   Ausgaenge:
%     x_ref,v_ref,a_ref 3x1 Soll-Pos/Geschw/Beschl 
%     yaw_ref     1x1 Soll-Yaw 
%     Omega_ref   3x1 Vorsteuer-Drehrate (Body) 
%     tau_ff      3x1 Vorsteuer-Moment (Body) 
%     q_ref       4x1 nominelles Soll-Quaternion
%     F_ref       1x1 nomineller Schubbetrag = m*||a_d - g|| [N] -- Debug
%

    g_grav = [0; 0; quadcop.g];
    N = size(traj.P, 2);

    % --- Phase bestimmen ---
    % Phasenfolge: dwell(1), move(1), dwell(2), move(2), ..., dwell(N)
    mode    = int8(2); % 0=Rast, 1=Bewegung, 2=End-Halt (Default)
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
            Dd = traj.Tdwell(i); % Rast an WP i
            if (t >= acc) && (t < acc + Dd)
                mode = int8(0); 
                sel_wp = i; 
                tloc = t - acc; 
                break;
            end
            acc = acc + Dd;
            if i < N
                Tm = traj.Tseg(i); % Bewegung WP i -> i+1
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
        T  = traj.Tseg(k);
        tau = tloc / T;
        [s0,s1,s2,s3,s4] = restpoly(tau);
        D   = traj.P(:,k+1) - traj.P(:,k); % Differenz zwischen den Wegpunkten
        x_ref = traj.P(:,k) + D*s0;
        v_ref = D*s1 / T;
        a_ref = D*s2 / T^2;
        j_ref = D*s3 / T^3;
        s_ref = D*s4 / T^4;
        yaw_ref = traj.yaw(k);
    elseif mode == int8(0) % Rastpunkt
        x_ref = traj.P(:,sel_wp);
        v_ref = zeros(3,1);  
        a_ref = zeros(3,1);
        j_ref = zeros(3,1);
        s_ref = zeros(3,1);
        if sel_wp < N
            yaw_ref = traj.yaw(sel_wp);    % Yaw des kommenden Segments
        else
            yaw_ref = traj.yaw(N-1);
        end
    else % End-Halt
        x_ref = traj.P(:,N);
        v_ref = zeros(3,1);  
        a_ref = zeros(3,1);
        j_ref = zeros(3,1);
        s_ref = zeros(3,1);
        yaw_ref = traj.yaw(N-1);
    end

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
    xC = [cos(yaw_ref); sin(yaw_ref); 0];

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
    q_ref  = dcm2quat_local(Rd.');            % Rd' = R_{b<-n}
end

% --- Lokale Helfer ---

% function [s0,s1,s2,s3,s4] = restpoly(tau)
% % Minimum-Snap rest-to-rest Einheitsprofil 0->1 ueber tau in [0,1].
% % s und erste drei Ableitungen = 0 an beiden Enden (Snap != 0 an den Enden!).
%     if tau < 0  
%         tau = 0; 
%     elseif tau > 1 
%         tau = 1; 
%     end
%     t2=tau*tau; 
%     t3=t2*tau; 
%     t4=t3*tau; 
%     t5=t4*tau; 
%     t6=t5*tau; 
%     t7=t6*tau;
%     s0 =  35*t4 -  84*t5 +  70*t6 -  20*t7;
%     s1 = 140*t3 - 420*t4 + 420*t5 - 140*t6;     % ds/dtau
%     s2 = 420*t2 -1680*t3 +2100*t4 - 840*t5;     % d2s/dtau2
%     s3 = 840*tau-5040*t2 +8400*t3 -4200*t4;     % d3s/dtau3
%     s4 = 840    -10080*tau+25200*t2-16800*t3;   % d4s/dtau4
% end
function [s0,s1,s2,s3,s4] = restpoly(tau)
% Minimum-Crackle rest-to-rest Einheitsprofil 0->1 ueber tau in [0,1].
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
% Shepperd, Skalar zuerst, R = Inertial->Koerper (R_{b<-n}).
% Muss vorzeichengleich zur Projektversion bleiben.
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
