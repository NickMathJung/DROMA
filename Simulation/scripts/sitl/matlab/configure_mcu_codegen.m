%% configure_mcu_codegen.m  --  Embedded-Coder-Config fuer mcu.slx (Onboard-MCU)
%  Erzeugt C++-Quelle plus packNGo-ZIP, kein Ziel-Compile aus MATLAB.
%  Voraussetzung: mcu.slx ist geladen, Ts_inner liegt im Base-Workspace.

function configure_mcu_codegen(mdl, target)
% target = 'host' (Default, SITL/x86, ConfigSet 'ert_cpp_sitl')
%        | 'arm'  (Teensy 4.1 / Cortex-M7, ConfigSet 'ert_cpp_arm')
% Gemeinsam: C++ class MCU, SingleTasking, DISCRETE Ts_inner, GenCodeOnly.
if nargin < 1, mdl = 'mcu'; end
if nargin < 2, target = 'host'; end
target = validatestring(target, {'host','arm'});
load_system(mdl);

% --- Basisrate im Base-Workspace ---
if ~evalin('base','exist(''Ts_inner'',''var'')')
    warning(['Ts_inner nicht im Base-Workspace. PreLoadFcn/params.m nicht ' ...
             'gelaufen? FixedStep=''Ts_inner'' wird sonst beim Build scheitern.']);
end

cs = getActiveConfigSet(mdl);
cs = copy(cs);
if strcmp(target,'arm'), cs.Name = 'ert_cpp_arm'; else, cs.Name = 'ert_cpp_sitl'; end

% --- Zielsprache / Target ---
set_param(cs,'SystemTargetFile','ert.tlc');
set_param(cs,'TargetLang','C++');
set_param(cs,'CodeInterfacePackaging','C++ class');
set_param(cs,'GenCodeOnly','on');
set_param(cs,'PackageGeneratedCodeAndArtifacts','on');   % packNGo-ZIP

% --- Solver ---
set_param(cs,'SolverType','Fixed-step');
set_param(cs,'Solver','FixedStepDiscrete');
set_param(cs,'FixedStep','Ts_inner');                    % 1/1000 s
set_param(cs,'SolverMode','SingleTasking');              % genau ein step()

% --- Embedded-clean ---
set_param(cs,'SupportNonFinite','off');
set_param(cs,'MatFileLogging','off');
set_param(cs,'GenerateReport','on');
set_param(cs,'GenerateComments','on');
set_param(cs,'ArrayLayout','Column-major');

% Auf HW: Compiler ohne -ffast-math, FPU round-to-nearest.

% --- Ziel-Hardware ---------------------------------------------------------
if strcmp(target,'arm')
    % Teensy 4.1: i.MX RT1062, Cortex-M7, little-endian, HW-DP-FPU -> double 64b.
    % Wortbreiten aus dem ARM-Preset (char8/short16/int32/long32/longlong64/
    % float32/double64/ptr32).
    set_param(cs,'ProdEqTarget','on');
    set_param(cs,'ProdHWDeviceType','ARM Compatible->ARM Cortex-M');
    set_param(cs,'ProdEndianess','LittleEndian');
    set_param(cs,'ProdLongLongMode','on');               % 64-bit long long
else
    % host: Default-Device (MATLAB-Host x86-64)
end

attachConfigSet(mdl, cs, true);
setActiveConfigSet(mdl, cs.Name);
fprintf('ConfigSet "%s" (target=%s) an %s gehaengt und aktiv.\n', cs.Name, target, mdl);

% --- Klassenname auf 'MCU' pinnen -----------------------------------------
try
    cm = coder.mapping.api.get(mdl);
catch
    cm = coder.mapping.utils.create(mdl);
end
setClassName(cm, 'MCU');
assert(strcmp(getClassName(cm),'MCU'), ...
       'Klassenname konnte nicht auf MCU gesetzt werden (getClassName=%s).', ...
       getClassName(cm));
fprintf('C++-Klassenname gepinnt: getClassName = "%s".\n', getClassName(cm));

% --- Entry-Point-Kontrakt der generierten C++-Klasse ----------------------
%   class MCU {
%     public: void initialize();
%             void step();          % 1 kHz Basisrate
%             void terminate();
%             % I/O ueber ExternalInputs (ExtU_MCU_T) / ExternalOutputs (ExtY_MCU_T)
%   };
fprintf(['\nNaechster Schritt:\n' ...
         '  1) slbuild(''%s'')            %% -> C++-Klasse MCU + packNGo-ZIP\n' ...
         '  2) log_mcu_golden.m           %% Golden-I/O an der MCU-Grenze aufzeichnen\n' ...
         '  3) mcu.h/mcu_types.h + Report %% an den Host-Harness (test_mcu_model) geben\n'], ...
         mdl);
end