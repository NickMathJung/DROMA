%% log_mcu_golden.m — Golden-I/O an der MCU-Blockgrenze aufzeichnen (Modell-SITL).
%  Laeuft den geschlossenen Kreis (Strecke + MCU) einmal in Simulink und schreibt
%  die Signale an der MCU-Grenze als breite CSV auf dem Basisraster (Ts_inner),
%  damit der Host-Harness den generierten step() tickweise dagegen diffen kann.
%
%  Spaltennamen = ExtU/ExtY-Feldpfade, der Host mappt sie 1:1:
%     in:  Bus_IMU, Bus_Cmd, batt_count
%     out: rotor_cmd, led
%  z.B.  Bus_Cmd.q_ref.1..4 , Bus_IMU.imu_gyro.1..3 , rotor_cmd.1..N , led.1
%
%  Prinzip (analog zur Leaf-Golden-Stufe):
%    - Line-Logging an In-/Out-Ports des MCU-Blocks, benannt nach Portname.
%    - sim() als normale Modell-Simulation, ohne Codegen.
%    - Busse rekursiv zu Skalar-Spalten flatten (Vektoren column-major .1 .2 ..).
%    - alles per Zero-Order-Hold auf t = 0:Ts_inner:Tstop (so haelt auch der
%      generierte step() langsame Eingaenge zwischen Updates).
%
%  --- Anpassen ---
TOP_MODEL = 'quadcop';
MCU_BLOCK = 'quadcop/running on the quadrocopter MCU';
T_STOP    = 5.0;                  % [s] Simulationsdauer
% Portnamen in genau der Blockport-Reihenfolge (== ExtU/ExtY-Feldnamen).
% Bestaetigt: rein Bus_IMU, Bus_Cmd, batt_count, btn_ack ; raus rotor_cmd, led.
IN_NAMES  = {'Bus_IMU','Bus_Cmd','batt_count','btn_ack'};
OUT_NAMES = {'rotor_cmd','led','throttle'};  % led=Batterie-FSM-state (uint8); throttle[4]=[0,100] (OneShot125-Vorstufe).
OUT_CSV   = fullfile(fileparts(mfilename('fullpath')),'..','data','golden_mcu_io.csv');

load_system(TOP_MODEL);
assert(evalin('base','exist(''Ts_inner'',''var'')'), ...
       'Ts_inner fehlt im Base-Workspace (params.m via PreLoadFcn?).');
Ts_inner = evalin('base','Ts_inner');

%% --- GS-Serial-Bloecke (Design A) fuer die headless Golden-Sim auskommentieren --
%  'Serial Configuration'/'Serial Send' oeffnen beim Sim-Start einen COM-Port und
%  scheitern headless mit "No ports selected". Sie sind GS-Ausgang und beruehren
%  die MCU-Grenze nicht. Also in-memory auskommentieren und nach der Sim
%  wiederherstellen; das Modell wird nicht gespeichert, die GS-Seite auf Disk
%  bleibt unangetastet.
serialBlks = find_system(TOP_MODEL,'LookUnderMasks','on','FollowLinks','on', ...
                         'RegExp','on','Name','[Ss]erial');
serialPrev = get_param(serialBlks,'Commented');
for b = 1:numel(serialBlks), set_param(serialBlks{b},'Commented','on'); end
if ~isempty(serialBlks)
    fprintf('log_mcu_golden: %d Serial-Block(e) fuer die Sim auskommentiert.\n', numel(serialBlks));
end

%% --- 1) Line-Logging an der MCU-Grenze aktivieren ---------------------------
ph  = get_param(MCU_BLOCK,'PortHandles');
assert(numel(ph.Inport)  == numel(IN_NAMES), ...
   'MCU hat %d Inports, IN_NAMES nennt %d — Reihenfolge/Anzahl pruefen.', ...
   numel(ph.Inport), numel(IN_NAMES));
assert(numel(ph.Outport) == numel(OUT_NAMES), ...
   'MCU hat %d Outports, OUT_NAMES nennt %d — Reihenfolge/Anzahl pruefen.', ...
   numel(ph.Outport), numel(OUT_NAMES));
tags = {};
tags = add_port_logging(ph.Inport,  IN_NAMES,  tags);
tags = add_port_logging(ph.Outport, OUT_NAMES, tags);

%% --- 2) Simulieren ----------------------------------------------------------
cs = getActiveConfigSet(TOP_MODEL);
prev = struct('SignalLogging', get_param(cs,'SignalLogging'), ...
              'SignalLoggingName', get_param(cs,'SignalLoggingName'));
set_param(cs,'SignalLogging','on','SignalLoggingName','logsout');
simOut = sim(TOP_MODEL, 'StopTime', num2str(T_STOP), ...
                        'SaveOutput','on','ReturnWorkspaceOutputs','on');
logs = simOut.get('logsout');

% Serial-Bloecke wiederherstellen (in-memory; Disk war nie betroffen).
for b = 1:numel(serialBlks), set_param(serialBlks{b},'Commented',serialPrev{b}); end

%% --- 3) Basisraster + Spalten sammeln --------------------------------------
t = (0:Ts_inner:T_STOP).';
cols = struct('name',{},'data',{});
for i = 1:size(tags,1)
    sig  = logs.getElement(tags{i,2});
    cols = flatten_and_zoh(sig.Values, tags{i,2}, t, cols);
end

%% --- 4) CSV schreiben (Kopf = Namen, dann k,t,<cols>) -----------------------
if ~isfolder(fileparts(OUT_CSV)); mkdir(fileparts(OUT_CSV)); end
fid = fopen(OUT_CSV,'w');  assert(fid>0, 'CSV nicht schreibbar: %s', OUT_CSV);
fprintf(fid, 'k,t%s\n', sprintf(',%s', cols.name));
M = numel(t);
for k = 1:M
    fprintf(fid, '%d,%.17g', k-1, t(k));
    for c = 1:numel(cols); fprintf(fid, ',%.17g', cols(c).data(k)); end
    fprintf(fid, '\n');
end
fclose(fid);
set_param(cs,'SignalLogging',prev.SignalLogging, ...
             'SignalLoggingName',prev.SignalLoggingName);

fprintf('\nGolden geschrieben: %s\n', OUT_CSV);
fprintf('  %d Ticks @ Ts_inner=%.6g s, %d Spalten.\n', M, Ts_inner, numel(cols));
fprintf('  Spalten: %s\n', strjoin({cols.name}, ', '));
fprintf(['\nWeiter: mcu.h/mcu_types.h + Codegen-Report an den Host-Harness ' ...
         '(test_mcu_model) — Spaltennamen mappen 1:1 auf ExtU/ExtY.\n']);


%% --- lokale Funktionen ---
function tags = add_port_logging(portHandles, names, tags)
% Signal-Logging haengt am erzeugenden Output-Port, nicht an der Linie:
%   - MCU-Ausgang  -> Quelle ist der MCU-Outport selbst
%   - MCU-Eingang  -> Quelle ist der speisende Block-Outport (SrcPortHandle)
% DataLogging/-NameMode/-Name sind Eigenschaften des Output-Port-Handles.
    for p = 1:numel(portHandles)
        ln = get_param(portHandles(p),'Line');
        if ln == -1
            error(['MCU-Port "%s" ist in quadcop unverdrahtet — kein Signal zum ' ...
                   'Loggen. Falls es der led-Ausgang ist: in quadcop einen ' ...
                   'Terminator/Scope an den led-Outport haengen, damit eine ' ...
                   'Linie existiert.'], names{p});
        end
        src = get_param(ln,'SrcPortHandle');   % erzeugender Output-Port
        set_param(src, 'DataLogging','on', ...
                       'DataLoggingNameMode','Custom', ...
                       'DataLoggingName', names{p});
        tags(end+1,:) = {src, names{p}}; %#ok<AGROW>
    end
end

function cols = flatten_and_zoh(vals, prefix, t, cols)
% Rekursiv: Bus (struct von timeseries) -> Skalar-Spalten, ZOH auf t.
    if isstruct(vals)
        fn = fieldnames(vals);
        for f = 1:numel(fn)
            cols = flatten_and_zoh(vals.(fn{f}), [prefix '.' fn{f}], t, cols);
        end
        return;
    end
    tt = vals.Time(:);
    n  = numel(tt);
    D  = double(reshape(vals.Data, n, []));  % [nSamples x nChannels]; cast (uint8/boolean -> double fuer interp1)
    for c = 1:size(D,2)
        cols(end+1) = struct('name', sprintf('%s.%d',prefix,c), ...
                             'data', zoh_resample(tt, D(:,c), t)); %#ok<AGROW>
    end
end

function y = zoh_resample(tt, x, tq)
% Index-basiertes ZOH, robust gegen FP-Drift zwischen Simulink-Zeit und Raster:
% jeden geloggten Sample auf seinen ganzzahligen Basistakt-Index runden, dann
% Luecken mit dem letzten gueltigen Wert fuellen. Kein interp1 auf
% Fliesskomma-Zeit, damit an Sample-Grenzen kein 'previous' danebengreift.
%   - Basisrate (1 kHz): jeder Tick hat ein Sample -> 1:1, keine Luecken.
%   - langsame Kanaele (z.B. 100 Hz): Sample alle 10 Ticks -> ZOH dazwischen.
    tt = double(tt(:)); x = double(x(:));
    N  = numel(tq);
    Ts = tq(2) - tq(1);
    y  = nan(N,1);
    ki = round(tt./Ts) + 1;                 % 1-basierter Basistakt-Index je Sample
    for j = 1:numel(ki)
        if ki(j) >= 1 && ki(j) <= N, y(ki(j)) = x(j); end
    end
    last = x(1);                            % vor dem ersten Sample: erster Wert
    for i = 1:N
        if isnan(y(i)), y(i) = last; else, last = y(i); end
    end
end