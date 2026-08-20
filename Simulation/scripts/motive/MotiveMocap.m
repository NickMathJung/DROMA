classdef MotiveMocap < matlab.System
%MOTIVEMOCAP  Simulink-Quelle fuer OptiTrack/Motive (NatNet) -> Bus_Mocap.
%
%   Liefert die Pose EINES Rigid Body als:
%     mocap_pos  (3x1, [m])
%     mocap_quat (4x1, scalar-first [w x y z])
%     valid      (bool)
%
%   Konventionen:
%   1) Quaternion: NatNet liefert qx,qy,qz,qw (scalar-last), hier umsortiert
%      auf scalar-first [w x y z].
%   2) Up-Axis: Motive streamt Z-Up (Settings -> Streaming -> "Up Axis" = Z).
%      Dieser Block transformiert nicht.
%   3) Einheit: NatNet liefert Meter, kein Skalieren noetig.
%
%   Inbetriebnahme:
%   - Motive: Streaming aktiv, "Up Axis"=Z, Rigid Body angelegt, dessen
%     Streaming-ID hier als StreamingID eintragen.
%   - HostIP = IP des Motive-Rechners, ClientIP = IP dieses Rechners. Auf
%     einem Rechner reicht 127.0.0.1/Multicast.
%
%   Der NatNet-Client ist .NET-basiert (NatNetML.dll) und nicht codegen-faehig,
%   der "MATLAB System"-Block laeuft deshalb interpretiert. Der DLL-Pfad kommt
%   aus Matlab\assemblypath.txt neben natnet.m.

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
            % Name-Value-Konstruktor
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = private)
        function s = resolveIP(~, v)
            %resolveIP  IP-Literal oder base-Workspace-Variable aufloesen.
            s = char(v);
            if isempty(regexp(s, '^[A-Za-z_]\w*(\.\w+)*$', 'once'))
                return; % IP-Literal oder Hostname mit Ziffern
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
                    fprintf(['[MotiveMocap] NICHT verbunden -> valid=false, ' ...
                             'Pose bleibt auf dem letzten Wert.\n']);
                end
            end
        end

        function [pos, quat, valid] = stepImpl(obj)
            pos   = obj.lastPos; % ZOH: letzten Wert halten
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
                    % NatNet: Meter, Quaternion scalar-last -> hier scalar-first
                    p = [double(rb.x); double(rb.y); double(rb.z)];
                    q = [double(rb.qw); double(rb.qx); double(rb.qy); double(rb.qz)];
                    nq = norm(q);
                    if nq < 0.5 || any(~isfinite(p)) || any(~isfinite(q))
                        return; % untracked/ungueltig -> ZOH
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
        % NatNet ist .NET und nicht codegen-faehig: Modus fest, Dialogoption aus
        function simMode = getSimulateUsingImpl()
            simMode = 'Interpreted execution';
        end
        function isVisible = showSimulateUsingImpl()
            isVisible = false;
        end
    end
end
