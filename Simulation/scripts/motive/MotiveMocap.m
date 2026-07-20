classdef MotiveMocap < matlab.System
%MOTIVEMOCAP  Simulink-Quelle fuer OptiTrack/Motive (NatNet) -> Bus_Mocap.
%
%   Liefert die Pose EINES Rigid Body als:
%     mocap_pos  (3x1, [m], Frame {A})
%     mocap_quat (4x1, SCALAR-FIRST [w x y z])
%     valid      (bool) - false, wenn kein frischer Frame mit StreamingID kam
%
%   Konventionen (wichtig, sonst stimmt die Pose nicht):
%   1) Quaternion: NatNet liefert qx,qy,qz,qw (scalar-last). Das Projekt nutzt
%      durchgaengig scalar-first [w x y z]. Die Umsortierung [qw; qx; qy; qz]
%      passiert nur hier, an einer Stelle. Bitte downstream nicht noch einmal
%      drehen, sonst wirkt die Korrektur doppelt.
%   2) Up-Axis: Motive muss auf Z-Up streamen (Settings -> Streaming ->
%      "Up Axis" = Z), passend zum z-up-Projekt. Dieser Block transformiert
%      absichtlich nicht, sonst gaebe es wieder zwei Stellen, die dasselbe
%      korrigieren. Steht Motive auf Y-Up, ist die Pose falsch; das faellt im
%      Plausibilitaets-Check unten auf.
%   3) Einheit: NatNet liefert Meter (das OptiSample multipliziert nur fuer die
%      mm-Anzeige mit 1000). Kein Skalieren noetig.
%
%   Inbetriebnahme:
%   - Motive: Streaming aktiv, "Up Axis"=Z, Rigid Body angelegt, dessen
%     Streaming-ID hier als StreamingID eintragen.
%   - Bei Verbindungsproblemen: HostIP = IP des Motive-Rechners, ClientIP = IP
%     dieses Rechners. Auf einem Rechner reicht 127.0.0.1/Multicast.
%   - Plausibilitaet (Drohne ruhig am Boden, Body-Origin am Boden kalibriert):
%       mocap_pos ~ [x; y; ~0]   und mocap_quat ~ [1;0;0;0] bei Nullrotation.
%     Zeigt stattdessen die Y-Komponente die Hoehe, steht Motive auf Y-Up.
%
%   Der NatNet-Client ist .NET-basiert (NatNetML.dll) und nicht codegen-faehig,
%   der "MATLAB System"-Block laeuft deshalb interpretiert (siehe
%   getSimulateUsingImpl weiter unten). Der DLL-Pfad kommt aus vorab
%   geschriebenem Matlab\assemblypath.txt neben natnet.m; sonst oeffnet
%   natnet.setAssemblyPath ein uigetfile-Fenster und blockiert die Simulation.

    properties (Nontunable)
        HostIP        = '127.0.0.1'   % IP des Motive-Rechners
        ClientIP      = '127.0.0.1'   % IP dieses Rechners
        StreamingID   = 1             % Streaming-ID des Rigid Body in Motive
        SampleTimeSec = 0.01          % == Ts_gcs
    end

    properties (Nontunable, Logical)
        Verbose = true                % Verbindungs-/Statusmeldungen
    end

    properties (Access = private)
        client
        connected  = false
        warnedNoRB = false
        lastPos    = [0;0;0]
        lastQuat   = [1;0;0;0]
    end

    methods
        function obj = MotiveMocap(varargin)
            % Name-Value-Konstruktor: den liefert matlab.System nicht von
            % selbst, ohne setProperties scheitert der Aufruf mit
            % "No matching constructor found for superclass".
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = private)
        function s = resolveIP(~, v)
            %resolveIP  IP-Literal oder base-Workspace-Variable aufloesen.
            % Simulink wertet numerische Dialogfelder eines MATLAB-System-Blocks
            % aus (StreamingID='mocap.streaming_id' -> 1), char-Felder aber nicht:
            % dort kam wortwoertlich 'mocap.host_ip' an. Damit params.m die
            % einzige Konfigurationsstelle bleibt (statt IPs im binaeren .slx zu
            % vergraben), loesen wir hier selbst auf:
            %   '127.0.0.1'      -> direkt (enthaelt Ziffern/Punkte)
            %   'mocap.host_ip'  -> evalin('base', ...)
            s = char(v);
            if isempty(regexp(s, '^[A-Za-z_]\w*(\.\w+)*$', 'once'))
                return;   % IP-Literal oder Hostname mit Ziffern -> unveraendert
            end
            if ~isempty(regexp(s, '^\d', 'once'))
                return;
            end
            try
                val = evalin('base', s);
                if ischar(val) || isstring(val)
                    s = char(val);
                end
            catch
                % nicht aufloesbar -> als Hostname durchreichen
            end
        end
    end

    methods (Access = protected)

        function setupImpl(obj)
            obj.connected = false;
            obj.lastPos   = [0;0;0];
            obj.lastQuat  = [1;0;0;0];
            obj.warnedNoRB = false;
            host   = obj.resolveIP(obj.HostIP);
            client = obj.resolveIP(obj.ClientIP);
            try
                obj.client = natnet();
                ok = obj.client.ConnectToNatNet(client, host, 'Multicast');
                obj.connected = (ok >= 1);
            catch ME
                obj.connected = false;
                warning('MotiveMocap:connect', ...
                    'NatNet-Verbindung fehlgeschlagen: %s', ME.message);
            end
            if obj.Verbose
                if obj.connected
                    fprintf('[MotiveMocap] verbunden (Host %s, ID %d)\n', ...
                            host, obj.StreamingID);
                else
                    % Kein Abbruch: die Sim laeuft mit valid=false weiter, damit
                    % der Pruefstand auch ohne Motive testbar bleibt.
                    fprintf(['[MotiveMocap] NICHT verbunden -> valid=false, ' ...
                             'Pose bleibt auf dem letzten Wert.\n']);
                end
            end
        end

        function [pos, quat, valid] = stepImpl(obj)
            pos   = obj.lastPos;      % ZOH: bei Aussetzern letzten Wert halten
            quat  = obj.lastQuat;
            valid = false;
            if ~obj.connected
                return;
            end
            try
                data = obj.client.getFrame();
                if isempty(data) || ~isprop(data,'RigidBodies') || data.nRigidBodies < 1
                    return;
                end
                for i = 1:data.nRigidBodies
                    rb = data.RigidBodies(i);
                    if rb.ID ~= obj.StreamingID
                        continue;
                    end
                    % NatNet: Meter, Quaternion scalar-last -> hier scalar-first.
                    p = [double(rb.x); double(rb.y); double(rb.z)];
                    q = [double(rb.qw); double(rb.qx); double(rb.qy); double(rb.qz)];
                    nq = norm(q);
                    if nq < 0.5 || any(~isfinite(p)) || any(~isfinite(q))
                        return;   % untracked/ungueltig -> ZOH, valid bleibt false
                    end
                    q = q / nq;
                    obj.lastPos = p; obj.lastQuat = q;
                    pos = p; quat = q; valid = true;
                    return;
                end
                if ~obj.warnedNoRB
                    warning('MotiveMocap:noRigidBody', ...
                        'Kein Rigid Body mit StreamingID=%d im Frame.', obj.StreamingID);
                    obj.warnedNoRB = true;
                end
            catch ME
                if ~obj.warnedNoRB
                    warning('MotiveMocap:getFrame', 'getFrame fehlgeschlagen: %s', ME.message);
                    obj.warnedNoRB = true;
                end
            end
        end

        function releaseImpl(obj)
            try
                if ~isempty(obj.client) && obj.connected
                    obj.client.disconnect();
                end
            catch
            end
            obj.connected = false;
        end

        % ---- Simulink-Schnittstelle ------------------------------------
        function num = getNumInputsImpl(~),  num = 0; end
        function num = getNumOutputsImpl(~), num = 3; end
        function varargout = getOutputSizeImpl(~)
            varargout = {[3 1], [4 1], [1 1]};
        end
        function varargout = getOutputDataTypeImpl(~)
            varargout = {'double', 'double', 'logical'};
        end
        function varargout = isOutputComplexImpl(~)
            varargout = {false, false, false};
        end
        function varargout = isOutputFixedSizeImpl(~)
            varargout = {true, true, true};
        end
        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', ...
                                        'SampleTime', obj.SampleTimeSec);
        end
    end

    methods (Static, Access = protected)
        % Der NatNet-Client ist .NET (NatNetML.dll) und nicht codegen-faehig
        % (schon try/catch scheitert mit "Try and catch are not supported for
        % code generation"). Der Modus wird deshalb hier erzwungen statt nur am
        % Block gesetzt, damit er nicht versehentlich auf 'Code generation'
        % zurueckfallen kann; die Option wird im Blockdialog ausgeblendet.
        function simMode = getSimulateUsingImpl()
            simMode = 'Interpreted execution';
        end
        function isVisible = showSimulateUsingImpl()
            isVisible = false;
        end
    end
end
