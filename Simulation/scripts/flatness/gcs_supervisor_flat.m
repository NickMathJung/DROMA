function [p_ref, v_ref, a_ref, j_ref, s_ref, yaw_ref, estop, mode] = ...
        gcs_supervisor_flat(estop_cmd, p_est, x_ref_traj, v_ref_traj, a_ref_traj, ...
                            j_ref_traj, s_ref_traj, yaw_ref_traj, supervisor)
%#codegen
% gcs_supervisor_flat  Zustandsautomat der Bodenstation — FLATNESS-Variante.
%
% Gleiche FSM wie gcs_supervisor (Kaskade), aber statt {x/v/a_ref + Lage-
% Vorsteuerung} werden die FLACHEN Referenzen bis Snap selektiert. Im
% Soft-Land/Kill werden j_ref/s_ref genullt (Rampe = konstante Geschwindigkeit).
% p_est = mocap_pos (der Beobachter laeuft in der Flatness-Variante onboard).
%
% Zustaende (mode):
%   0 NORMAL     : Referenzen aus der Trajektorie, estop=0.
%   1 SOFT_LAND  : x/y einfrieren, z-Ref rampt mit v_sink runter, estop=1.
%   2 DISARMED   : Grund erreicht, estop=2, onboard-Cutoff.
%   3 KILL       : Hard-Kill (estop_cmd==2 aus jedem Zustand), estop=2.
%
% Eingaenge:
%   estop_cmd    : uint8  Bediener-Wunsch 0 normal / 1 soft-land / 2 hard-kill
%   p_est        : 3x1    Positionsmessung (mocap_pos)
%   x_ref_traj..s_ref_traj : 3x1  flache Referenzen aus traj_gen
%   yaw_ref_traj : 3x1    [yaw; dyaw; ddyaw]
%   supervisor   : struct .v_sink .z_ground .disarm_margin .Ts
% Ausgaenge -> Bus_Cmd_flat.

NORMAL = uint8(0);
SOFT_LAND = uint8(1);
DISARMED = uint8(2);
KILL = uint8(3);

persistent state x0 y0 yaw0 zref inited
if isempty(inited)
    state = NORMAL;
    x0 = 0.0;
    y0 = 0.0;
    yaw0 = [0.0; 0.0; 0.0];
    zref = 0.0;
    inited = true;
end

% --- Hard-Kill gewinnt immer, aus jedem Zustand ---
if estop_cmd == uint8(2)
    state = KILL;
end

% --- Transitionen + zustandslokale Aktualisierung ---
switch state
    case NORMAL
        if estop_cmd == uint8(1)          % Soft-Land ausloesen
            state = SOFT_LAND;
            x0 = p_est(1);                % Horizontalposition einfrieren
            y0 = p_est(2);
            yaw0 = yaw_ref_traj;          % aktuellen Soll-Yaw halten
            zref = x_ref_traj(3);         % z-Rampe startet auf aktueller Hoehe
        end

    case SOFT_LAND
        zref = zref - supervisor.v_sink * supervisor.Ts;
        if zref < supervisor.z_ground
            zref = supervisor.z_ground;
        end
        if p_est(3) <= supervisor.z_ground + supervisor.disarm_margin
            state = DISARMED;
        end

    case DISARMED
        % terminal: estop=2 nullt onboard die Motoren

    otherwise % KILL
        % terminal
end

% --- Ausgangs-Mux nach Zustand ---
switch state
    case NORMAL
        p_ref   = x_ref_traj;
        v_ref   = v_ref_traj;
        a_ref   = a_ref_traj;
        j_ref   = j_ref_traj;
        s_ref   = s_ref_traj;
        yaw_ref = yaw_ref_traj;
        estop   = uint8(0);

    case SOFT_LAND
        p_ref   = [x0; y0; zref];
        v_ref   = [0.0; 0.0; -supervisor.v_sink];
        a_ref   = [0.0; 0.0; 0.0];
        j_ref   = [0.0; 0.0; 0.0];
        s_ref   = [0.0; 0.0; 0.0];
        yaw_ref = yaw0;
        estop   = uint8(1);

    case DISARMED
        p_ref   = [x0; y0; zref];
        v_ref   = [0.0; 0.0; 0.0];
        a_ref   = [0.0; 0.0; 0.0];
        j_ref   = [0.0; 0.0; 0.0];
        s_ref   = [0.0; 0.0; 0.0];
        yaw_ref = yaw0;
        estop   = uint8(2);

    otherwise % KILL
        p_ref   = [x0; y0; zref];
        v_ref   = [0.0; 0.0; -supervisor.v_sink];
        a_ref   = [0.0; 0.0; 0.0];
        j_ref   = [0.0; 0.0; 0.0];
        s_ref   = [0.0; 0.0; 0.0];
        yaw_ref = yaw0;
        estop   = uint8(2);
end

mode = state;
end
