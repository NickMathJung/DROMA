%% sil_check_mcu.m  --  Aequivalenz-Check Normal-Mode gegen SIL-Mode
%  Interaktiv in der MATLAB-IDE fahren, nicht headless.
%
%  Laesst den MCU-Model-Block einmal im Normal- und einmal im SIL-Mode laufen
%  (im SIL den generierten C++-Code) und vergleicht rotor_cmd, led und throttle.
%
%  Voraussetzung: MCU ist als Model-Block (referenziert 'mcu') im Harness, das
%  ert_cpp_sitl-ConfigSet ist aktiv. SIL baut den Code bei Bedarf selbst.
function sil_check_mcu(harness, mcuBlock)
if nargin < 1 
    harness  = 'quadcop';
end
if nargin < 2 
    mcuBlock = 'quadcop/running on the quadrocopter MCU';
end
load_system(harness);
Ts_inner = evalin('base','Ts_inner');
T_STOP   = 5.0;

% GS-Serial-Bloecke fuer die Sim auskommentieren, onCleanup stellt sie wieder her.
serialBlks = find_system(harness,'LookUnderMasks','on','FollowLinks','on', ...
                         'RegExp','on','Name','[Ss]erial');
serialPrev = get_param(serialBlks,'Commented');
for b = 1:numel(serialBlks), set_param(serialBlks{b},'Commented','on'); end
serialCleanup = onCleanup(@() cellfun(@(bl,st) set_param(bl,'Commented',st), ...
                                      serialBlks, serialPrev, 'UniformOutput', false)); %#ok<NASGU>

% rotor_cmd-Linie (erster Outport des MCU-Blocks) loggen.
ph = get_param(mcuBlock,'PortHandles');
outNames = {'rotor_cmd','led','throttle'};   % Reihenfolge der MCU-Outports
for oIdx = 1:numel(ph.Outport)
    if get_param(ph.Outport(oIdx),'Line') == -1
        warning('MCU-Ausgang "%s" unverdrahtet in quadcop — wird nicht geloggt.', ...
                outNames{oIdx});
        continue;
    end
    set_param(ph.Outport(oIdx),'DataLogging','on', ...
              'DataLoggingNameMode','Custom','DataLoggingName',outNames{oIdx});
end
cs = getActiveConfigSet(harness);
set_param(cs,'SignalLogging','on','SignalLoggingName','logsout');

modeWas = get_param(mcuBlock,'SimulationMode');
cleanup = onCleanup(@() set_param(mcuBlock,'SimulationMode',modeWas));

% --- Normal ---
set_param(mcuBlock,'SimulationMode','Normal');
N = sim(harness,'StopTime',num2str(T_STOP),'ReturnWorkspaceOutputs','on');
rN = grab(N,'rotor_cmd',Ts_inner,T_STOP);  lN = grab(N,'led',Ts_inner,T_STOP);
tN = grab(N,'throttle',Ts_inner,T_STOP);
% --- SIL: generierter Code im Loop ---
set_param(mcuBlock,'SimulationMode','Software-in-the-loop (SIL)');
S = sim(harness,'StopTime',num2str(T_STOP),'ReturnWorkspaceOutputs','on');
rS = grab(S,'rotor_cmd',Ts_inner,T_STOP);  lS = grab(S,'led',Ts_inner,T_STOP);
tS = grab(S,'throttle',Ts_inner,T_STOP);

if isempty(rN) || isempty(rS)
    error('rotor_cmd nicht geloggt — MCU-Ausgang in quadcop verdrahtet?');
end
w = max(abs(rN(:)-rS(:)));
fprintf('\n[SIL vs Normal] rotor_cmd max|d| = %.3e  ueber %d Ticks x %d Rotoren.\n', ...
        w, size(rN,1), size(rN,2));
if ~isempty(tN) && ~isempty(tS)
    fprintf('[SIL vs Normal] throttle  max|d| = %.3e  ueber %d Ticks x %d Rotoren.\n', ...
            max(abs(tN(:)-tS(:))), size(tN,1), size(tN,2));
end
led_ok = true;
if isempty(lN) || isempty(lS)
    fprintf(['[SIL vs Normal] led      = uebersprungen (Ausgang in quadcop ' ...
             'unverdrahtet). Fuer Gate B Terminator an led haengen.\n']);
else
    nl = sum(lN(:) ~= lS(:)); led_ok = (nl == 0);
    fprintf('[SIL vs Normal] led      Mismatches = %d von %d Ticks.\n', nl, numel(lN));
end
if w < 1e-9 && led_ok
    fprintf('  -> bit-nah. Weiter zu (B) Host-Golden fuer den strengen Nachweis.\n');
else
    fprintf(['  -> Abweichung. Klein & beschraenkt = ok (Feedback-Drift); ' ...
             'waechst sie auf, erst im Host-Test (B) tickgenau lokalisieren.\n']);
end
end

function Y = grab(simOut, name, Ts, T)
% Signal <name> aus logsout ziehen und per ZOH aufs Basisraster bringen.
% Fehlt das Element, ist das Ergebnis [].
    logs = simOut.get('logsout');
    sig = [];
    try, sig = logs.getElement(name); catch, sig = []; end
    if isempty(sig) || ~isa(sig,'Simulink.SimulationData.Signal')
        Y = []; return;
    end
    tt = sig.Values.Time(:);
    D  = double(reshape(sig.Values.Data, numel(tt), []));
    t  = (0:Ts:T).';
    Y  = zeros(numel(t), size(D,2));
    for c = 1:size(D,2)
        Y(:,c) = interp1(tt, D(:,c), t, 'previous', 'extrap');
        Y(t<tt(1),c) = D(1,c);
    end
end