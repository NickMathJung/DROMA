classdef MotiveMocapMulti < matlab.System
%MOTIVEMOCAPMULTI  Simulink-Quelle fuer n Rigid Bodies aus einem NatNet-Frame.
%
%   Ausgaenge: pos (3xn), quat (4xn, scalar-first), valid (nx1 logical).
%   Spalte k gehoert zu StreamingIDs(k). Ein Client, ein getFrame pro Tick.
%   Konventionen: Z-Up, Quaternion scalar-first [w x y z], Meter.

    properties (Nontunable)
        HostIP        = '127.0.0.1'
        ClientIP      = '127.0.0.1'
        StreamingIDs  = [1 2]         % Streaming-IDs der Rigid Bodies (1xn)
        SampleTimeSec = 0.01          % == Ts_gcs
    end
    properties (Nontunable, Logical)
        Verbose = true
    end
    properties (Access = private)
        client
        connected = false
        warnedNoRB = false
        lastPos
        lastQuat
    end

    methods
        function obj = MotiveMocapMulti(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = private)
        function s = resolveIP(~, v)
            % IP-Literal oder base-Workspace-Variable aufloesen
            s = char(v);
            if isempty(regexp(s, '^[A-Za-z_]\w*(\.\w+)*$', 'once')), return; end
            if ~isempty(regexp(s, '^\d', 'once')), return; end
            try
                val = evalin('base', s);
                if ischar(val) || isstring(val), s = char(val); end
            catch
            end
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            n = numel(obj.StreamingIDs);
            obj.lastPos  = zeros(3, n);
            obj.lastQuat = repmat([1;0;0;0], 1, n);
            obj.connected = false;
            obj.warnedNoRB = false;
            host   = obj.resolveIP(obj.HostIP);
            client = obj.resolveIP(obj.ClientIP);
            try
                obj.client = natnet();
                ok = obj.client.ConnectToNatNet(client, host, 'Multicast');
                obj.connected = (ok >= 1);
            catch ME
                obj.connected = false;
                warning('MotiveMocapMulti:connect', ...
                    'NatNet-Verbindung fehlgeschlagen: %s', ME.message);
            end
            if obj.Verbose
                if obj.connected
                    fprintf('[MotiveMocapMulti] verbunden (Host %s, IDs %s)\n', ...
                            host, mat2str(obj.StreamingIDs));
                else
                    fprintf(['[MotiveMocapMulti] NICHT verbunden -> valid=false, ' ...
                             'Posen bleiben auf dem letzten Wert.\n']);
                end
            end
        end

        function [pos, quat, valid] = stepImpl(obj)
            n = numel(obj.StreamingIDs);
            pos = obj.lastPos; quat = obj.lastQuat; valid = false(n, 1);
            if ~obj.connected, return; end
            try
                data = obj.client.getFrame();
                if isempty(data) || ~isprop(data,'RigidBodies') || data.nRigidBodies < 1
                    return;
                end
                for i = 1:data.nRigidBodies
                    rb = data.RigidBodies(i);
                    k = find(obj.StreamingIDs == rb.ID); % alle Spalten dieser id
                    if isempty(k), continue; end
                    p = [double(rb.x); double(rb.y); double(rb.z)];
                    q = [double(rb.qw); double(rb.qx); double(rb.qy); double(rb.qz)];
                    nq = norm(q);
                    if nq < 0.5 || any(~isfinite(p)) || any(~isfinite(q))
                        continue; % untracked -> ZOH fuer diesen Body
                    end
                    q = q / nq;
                    obj.lastPos(:,k)  = repmat(p, 1, numel(k));
                    obj.lastQuat(:,k) = repmat(q, 1, numel(k));
                    pos(:,k)  = repmat(p, 1, numel(k));
                    quat(:,k) = repmat(q, 1, numel(k));
                    valid(k) = true;
                end
                if ~any(valid) && ~obj.warnedNoRB
                    warning('MotiveMocapMulti:noRigidBody', ...
                        'Kein Rigid Body mit IDs %s im Frame.', mat2str(obj.StreamingIDs));
                    obj.warnedNoRB = true;
                end
            catch ME
                if ~obj.warnedNoRB
                    warning('MotiveMocapMulti:getFrame', 'getFrame fehlgeschlagen: %s', ME.message);
                    obj.warnedNoRB = true;
                end
            end
        end

        function releaseImpl(obj)
            try
                if ~isempty(obj.client) && obj.connected, obj.client.disconnect(); end
            catch
            end
            obj.connected = false;
        end

        function num = getNumInputsImpl(~),  num = 0; end
        function num = getNumOutputsImpl(~), num = 3; end
        function varargout = getOutputSizeImpl(obj)
            n = numel(obj.StreamingIDs);
            varargout = {[3 n], [4 n], [n 1]};
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
        function simMode = getSimulateUsingImpl()
            simMode = 'Interpreted execution'; % NatNet = .NET, kein Codegen
        end
        function isVisible = showSimulateUsingImpl()
            isVisible = false;
        end
    end
end
