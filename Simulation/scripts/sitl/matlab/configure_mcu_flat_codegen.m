%% configure_mcu_flat_codegen.m  --  Embedded-Coder-Config fuer mcu_flat.slx
%  ert.tlc / C++ class MCU_FLAT / SingleTasking / DISCRETE Ts_inner /
%  GenCodeOnly / column-major, ConfigSets 'ert_cpp_sitl_flat' und 'ert_cpp_arm_flat'.

function configure_mcu_flat_codegen(mdl, target)
% target = 'host' (Default, SITL/x86) | 'arm' (Teensy 4.1 / Cortex-M7)
if nargin < 1, mdl = 'mcu_flat'; end
if nargin < 2, target = 'host'; end
target = validatestring(target, {'host','arm'});
load_system(mdl);

if ~evalin('base','exist(''Ts_inner'',''var'')')
    warning(['Ts_inner nicht im Base-Workspace. PreLoadFcn/params.m nicht ' ...
             'gelaufen? FixedStep=''Ts_inner'' wird sonst beim Build scheitern.']);
end

cs = getActiveConfigSet(mdl);
cs = copy(cs);
if strcmp(target,'arm'), cs.Name = 'ert_cpp_arm_flat'; else, cs.Name = 'ert_cpp_sitl_flat'; end

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

% --- Ziel-Hardware ---------------------------------------------------------
if strcmp(target,'arm')
    set_param(cs,'ProdEqTarget','on');
    set_param(cs,'ProdHWDeviceType','ARM Compatible->ARM Cortex-M');
    set_param(cs,'ProdEndianess','LittleEndian');
    set_param(cs,'ProdLongLongMode','on');
end

attachConfigSet(mdl, cs, true);
setActiveConfigSet(mdl, cs.Name);
fprintf('ConfigSet "%s" (target=%s) an %s gehaengt und aktiv.\n', cs.Name, target, mdl);

% --- Klassenname auf 'MCU_FLAT' pinnen ------------------------------------
try
    cm = coder.mapping.api.get(mdl);
catch
    cm = coder.mapping.utils.create(mdl);
end
setClassName(cm, 'MCU_FLAT');
assert(strcmp(getClassName(cm),'MCU_FLAT'), ...
       'Klassenname konnte nicht auf MCU_FLAT gesetzt werden (getClassName=%s).', ...
       getClassName(cm));
fprintf('C++-Klassenname gepinnt: getClassName = "%s".\n', getClassName(cm));

fprintf(['\nNaechster Schritt:\n' ...
         '  1) slbuild(''%s'')                 %% -> C++-Klasse MCU_FLAT + packNGo-ZIP\n' ...
         '  2) log_mcu_flat_golden.m           %% Golden-I/O an der mcu_flat-Grenze\n' ...
         '  3) mcu_flat.h + Report             %% an den Host-Harness (test_mcu_flat_model)\n'], ...
         mdl);
end
