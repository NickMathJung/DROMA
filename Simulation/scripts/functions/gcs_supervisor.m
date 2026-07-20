function [x_ref, v_ref, a_ref, yaw_ref, Omega_ref, tau_ref, estop, mode] = ...
        gcs_supervisor(estop_cmd, p_est, x_ref_traj, v_ref_traj, a_ref_traj, ...
                       yaw_ref_traj, tau_ref_traj, supervisor, Omega_ref_traj)
%#codegen
% gcs_supervisor  Zustandsautomat der Bodenstation.
%
% Mux vor pos_ctrl: waehlt die Sollwertquelle (Trajektorie oder geregelter
% Soft-Land) und setzt das estop-Feld des Bus_Cmd.
%
% Zustaende (mode):
%   0 NORMAL     : Sollwerte aus der Trajektorie, estop=0.
%   1 SOFT_LAND  : x/y einfrieren, z-Ref rampt mit v_sink runter, v_ref=+v_sink,
%                  yaw halten. estop=1 (onboard sieht das Soft-Land-Flag).
%   2 DISARMED   : Grund erreicht, estop=2, onboard-Cutoff (rotors_cmd=0).
%   3 KILL       : Hard-Kill (estop_cmd==2 aus jedem Zustand), estop=2.
%
% Eingaenge:
%   estop_cmd       : uint8  Bediener-Wunsch  0 normal / 1 soft-land / 2 hard-kill
%   p_est           : 3x1    Positionsschaetzung [x;y;z] aus Luenberger
%   x_ref_traj      : 3x1    Trajektorien-Sollposition (Durchleitung in NORMAL)
%   v_ref_traj      : 3x1    Trajektorien-Sollgeschwindigkeit
%   a_ref_traj      : 3x1    Trajektorien-Sollbeschleunigung
%   yaw_ref_traj    : double Trajektorien-Soll-Yaw 
%   Omega_ref_traj  : 3x1    Trajektorien-Solldrehrate2
%   tau_ref_traj    : 3x1    Trajektorien-Sollmomente 
%   sup             : struct .v_sink .z_ground .disarm_margin .Ts
%
% Ausgaenge (-> pos_ctrl bzw. Bus_Cmd):
%   x_ref, v_ref, a_ref : 3x1    selektierte Sollwerte fuer pos_ctrl
%   yaw_ref             : double selektierter Soll-Yaw
%   Omega_ref, tau_ref  : 3x1    Lage-Vorsteuerung 
%   estop               : uint8  0/1/2 -> Bus_Cmd.estop (Uplink)
%   mode                : uint8  Zustands-ID (Logging/Debug)

% --- Zustands-IDs ---
NORMAL = uint8(0);
SOFT_LAND = uint8(1);
DISARMED = uint8(2);
KILL = uint8(3);

persistent state x0 y0 yaw0 zref inited
if isempty(inited)
    state = NORMAL;
    x0 = 0.0;   
    y0 = 0.0;   
    yaw0 = 0.0;
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
        if estop_cmd == uint8(1) % Soft-Land ausloesen
            state = SOFT_LAND;
            x0 = p_est(1); % Horizontalposition einfrieren
            y0 = p_est(2);
            yaw0 = yaw_ref_traj; % aktuellen Soll-Yaw halten
            zref = x_ref_traj(3); % z-Rampe startet auf aktueller Hoehe
        end

    case SOFT_LAND
        zref = zref - supervisor.v_sink * supervisor.Ts;
        if zref < supervisor.z_ground 
            zref = supervisor.z_ground;
        end
        % Disarm, sobald knapp ueber Grund (Mocap/Luenberger kennt Hoehe)
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
        x_ref   = x_ref_traj;
        v_ref   = v_ref_traj;
        a_ref   = a_ref_traj;
        yaw_ref = yaw_ref_traj;
        Omega_ref = Omega_ref_traj;   
        tau_ref   = tau_ref_traj;
        estop   = uint8(0);

    case SOFT_LAND
        x_ref   = [x0; y0; zref];       
        v_ref   = [0.0; 0.0; -supervisor.v_sink];
        a_ref   = [0.0; 0.0; 0.0];
        yaw_ref = yaw0;
        Omega_ref = [0.0; 0.0; 0.0];       
        tau_ref   = [0.0; 0.0; 0.0];
        estop   = uint8(1);

    case DISARMED % on-board-kill übernimmt
        x_ref   = [x0; y0; zref];
        v_ref   = [0.0; 0.0; supervisor.v_sink];
        a_ref   = [0.0; 0.0; 0.0];
        yaw_ref = yaw0;
        Omega_ref = [0.0; 0.0; 0.0];
        tau_ref   = [0.0; 0.0; 0.0];
        estop   = uint8(2);              

    otherwise % KILL
        x_ref   = [x0; y0; zref];                
        v_ref   = [0.0; 0.0; -supervisor.v_sink];
        a_ref   = [0.0; 0.0; 0.0];
        yaw_ref = yaw0;
        Omega_ref = [0.0; 0.0; 0.0];
        tau_ref   = [0.0; 0.0; 0.0];
        estop   = uint8(2);
end

mode = state;
end