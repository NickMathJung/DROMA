%% log_mcu_flat_golden.m  --  Golden-I/O an der mcu_flat-Blockgrenze aufzeichnen
%  Simuliert den geschlossenen Kreis (quadcop_flat) einmal und schreibt die Signale
%  an der mcu_flat-Grenze als breite CSV auf dem Basisraster Ts_inner.
%
%  Spaltennamen = ExtU/ExtY-Feldpfade:
%     in:  Bus_IMU, Bus_Cmd_flat, batt_count, btn_ack
%     out: rotor_cmd, led, throttle
%
%  --- Anpassen ---
TOP_MODEL = 'quadcop_flat';
MCU_BLOCK = 'quadcop_flat/mcu_flat_ref';
T_STOP    = 5.0;                  % [s] Simulationsdauer
IN_NAMES  = {'Bus_IMU','Bus_Cmd_flat','batt_count','btn_ack'};
% dbg = [k_hat; F; aint(3); u_fb_raw(3)]
OUT_NAMES = {'rotor_cmd','led','throttle','dbg'};
OUT_CSV   = fullfile(fileparts(mfilename('fullpath')),'..','data','golden_mcu_flat_io.csv');

load_system(TOP_MODEL);
assert(evalin('base','exist(''Ts_inner'',''var'')'), ...
       'Ts_inner fehlt im Base-Workspace (params.m via PreLoadFcn?).');
Ts_inner = evalin('base','Ts_inner');

%% --- Serial-Bloecke fuer die headless Sim auskommentieren -------------------
serialBlks = find_system(TOP_MODEL,'LookUnderMasks','on','FollowLinks','on', ...
                         'RegExp','on','Name','[Ss]erial');
serialPrev = get_param(serialBlks,'Commented');
for b = 1:numel(serialBlks), set_param(serialBlks{b},'Commented','on'); end
if ~isempty(serialBlks)
    fprintf('log_mcu_flat_golden: %d Serial-Block(e) fuer die Sim auskommentiert.\n', numel(serialBlks));
end

%% --- 1) Line-Logging an der mcu_flat-Grenze aktivieren ----------------------
ph  = get_param(MCU_BLOCK,'PortHandles');
assert(numel(ph.Inport)  == numel(IN_NAMES), ...
   'mcu_flat hat %d Inports, IN_NAMES nennt %d — Reihenfolge/Anzahl pruefen.', ...
   numel(ph.Inport), numel(IN_NAMES));
assert(numel(ph.Outport) == numel(OUT_NAMES), ...
   'mcu_flat hat %d Outports, OUT_NAMES nennt %d — Reihenfolge/Anzahl pruefen.', ...
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

for b = 1:numel(serialBlks), set_param(serialBlks{b},'Commented',serialPrev{b}); end

%% --- 3) Basisraster + Spalten sammeln --------------------------------------
t = (0:Ts_inner:T_STOP).';
cols = struct('name',{},'data',{});
for i = 1:size(tags,1)
    sig  = logs.getElement(tags{i,2});
    cols = flatten_and_zoh(sig.Values, tags{i,2}, t, cols);
end

%% --- 4) CSV schreiben: Kopf = Namen, dann k,t,<cols> ------------------------
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

%% --- lokale Funktionen ---
function tags = add_port_logging(portHandles, names, tags)
    for p = 1:numel(portHandles)
        ln = get_param(portHandles(p),'Line');
        if ln == -1
            error(['mcu_flat-Port "%s" ist in quadcop_flat unverdrahtet — kein ' ...
                   'Signal zum Loggen (Terminator an den Port haengen).'], names{p});
        end
        src = get_param(ln,'SrcPortHandle');
        set_param(src, 'DataLogging','on', ...
                       'DataLoggingNameMode','Custom', ...
                       'DataLoggingName', names{p});
        tags(end+1,:) = {src, names{p}}; %#ok<AGROW>
    end
end

function cols = flatten_and_zoh(vals, prefix, t, cols)
    if isstruct(vals)
        fn = fieldnames(vals);
        for f = 1:numel(fn)
            cols = flatten_and_zoh(vals.(fn{f}), [prefix '.' fn{f}], t, cols);
        end
        return;
    end
    tt = vals.Time(:);
    n  = numel(tt);
    % Simulink legt Data je nach Signal als N x W, W x N oder W x 1 x N ab.
    D = squeeze(vals.Data);
    if isvector(D)
        D = D(:);
    elseif size(D,1) ~= n && size(D,2) == n
        D = D.';
    end
    assert(size(D,1) == n, ...
        'Signal "%s": %d Zeitpunkte, Data ist %s — Orientierung unklar.', ...
        prefix, n, mat2str(size(vals.Data)));
    D = double(D);
    for c = 1:size(D,2)
        cols(end+1) = struct('name', sprintf('%s.%d',prefix,c), ...
                             'data', zoh_resample(tt, D(:,c), t)); %#ok<AGROW>
    end
end

function y = zoh_resample(tt, x, tq)
% Index-basiertes ZOH ueber round(t/Ts).
    tt = double(tt(:)); x = double(x(:));
    N  = numel(tq);
    Ts = tq(2) - tq(1);
    y  = nan(N,1);
    ki = round(tt./Ts) + 1;
    for j = 1:numel(ki)
        if ki(j) >= 1 && ki(j) <= N, y(ki(j)) = x(j); end
    end
    last = x(1);
    for i = 1:N
        if isnan(y(i)), y(i) = last; else, last = y(i); end
    end
end
