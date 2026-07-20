%% gen_lib_codegen.m — Codegen der mcu.slx-Leaf-Funktionen als linkbare C-Quellen.
%  Erzeugt je Funktion reine C-Source (kein Build), die der C++-Golden-Test direkt
%  einbindet (CMake -DQUAT_IMPL=codegen -DSAFETY_IMPL=codegen).
%
%  Das ist etwas anderes als:
%   (a) verify_quat_codegen.m: baut ein MEX (in MATLAB laufender Binaer) fuer den
%       MATLAB<->Coder-Check und laeuft nur in MATLAB.
%   (b) mcu.slx-Modell-Codegen (configure_mcu_codegen.m): absorbiert die Leafs in
%       eine C++-Klasse 'MCU', ohne standalone-Entry-Points je Funktion.
%  Diese Datei erzeugt jede Leaf-Funktion als eigene standalone C-Quelle, damit der
%  C++-Test ihre reine Numerik isoliert gegen die Golden-Vektoren diffen kann.
%
%  Speicherordnung: Coder legt Matrizen column-major ab. dcm2quat_local(R) erwartet
%  R daher column-major, quat2dcm_local(q) liefert R column-major. Genau darauf sind
%  include/quat_helpers.h und test/csv.hpp (row->col-Adapter) ausgelegt.
%
%  Pfade:
%  Erwartet: dieses Skript liegt in  Simulation\scripts\sitl\matlab\
%  Quellen in:                       Simulation\scripts\functions\
%  Ausgabe nach:                     Simulation\scripts\sitl\codegen\lib\<fn>\
%
%  Alle codegen-Aufrufe sind in Funktions-Syntax (codegen('name',...)); das
%  vermeidet die Whitespace-Fallen der Command-Syntax (z.B. '}-d' als Subtraktion).

clear; clc;
here         = fileparts(mfilename('fullpath'));            % ...\sitl\matlab
sitlRoot     = fileparts(here);                             % ...\sitl
functionsDir = fullfile(sitlRoot,'..','functions');        % ...\scripts\functions
outRoot      = fullfile(sitlRoot,'codegen','lib');         % == CODEGEN_ROOT/lib

assert(isfolder(functionsDir), 'functions-Ordner nicht gefunden: %s', functionsDir);
addpath(functionsDir);
if ~exist(outRoot,'dir'); mkdir(outRoot); end

cfg = coder.config('lib');
cfg.TargetLang       = 'C';       % triviale extern-"C"-Bindung; Golden-Diff = reine Numerik
cfg.GenCodeOnly      = true;      % nur Quelle; Compile macht CMake
cfg.GenerateReport   = true;
cfg.SupportNonFinite = false;     % embedded-clean
cfg.EnableOpenMP     = false;

%% ---- Quaternion-Helfer (stateless) ----
codegen('dcm2quat_local', '-config', cfg, '-args', {zeros(3,3)}, ...
        '-d', fullfile(outRoot,'dcm2quat_local'));
codegen('quat2dcm_local', '-config', cfg, '-args', {zeros(4,1)}, ...
        '-d', fullfile(outRoot,'quat2dcm_local'));
codegen('quatMul', '-config', cfg, '-args', {zeros(4,1), zeros(4,1)}, ...
        '-d', fullfile(outRoot,'quatMul'));
codegen('quatConj', '-config', cfg, '-args', {zeros(4,1)}, ...
        '-d', fullfile(outRoot,'quatConj'));
codegen('quatRotate', '-config', cfg, '-args', {zeros(4,1), zeros(3,1)}, ...
        '-d', fullfile(outRoot,'quatRotate'));

%% ---- Safety-Leafs (persistent state) ----
% Coder emittiert je <fn>_initialize()/<fn>()/<fn>_terminate(). Der duenne Shim
% (README, "Codegen-Shim") mappt reset()->*_initialize, step()->*.
os_p = struct('omega_max',10.0,'debounce_N',4.0,'use_norm',false, ...
              'tilt_cos_min',cosd(80),'tilt_debounce_N',80.0);
bt_p = struct('batt_k',3.3*18.182/4095,'batt_b',0.0,'batt_alpha',0.014, ...
              'V_warn',14.0,'V_crit',13.4,'V_floor',12.0,'V_hyst',0.2);
codegen('safety_overspeed', '-config', cfg, ...
        '-args', {zeros(3,1), zeros(4,1), uint8(0), false, false, coder.Constant(os_p)}, ...
        '-d', fullfile(outRoot,'safety_overspeed'));
codegen('safety_battery', '-config', cfg, ...
        '-args', {0.0, coder.Constant(bt_p)}, ...
        '-d', fullfile(outRoot,'safety_battery'));

%% ---- Noch nicht im Golden-Test abgedeckt (restliche mcu-Funktionen) ----
% geo_attitude_ctrl.m und mahony_filter.m liegen auf Modell-Ebene (mcu.slx);
% safety_landcmd.m ist als Leaf-Fixture geplant.

fprintf('\nGeneriert nach %s\n', outRoot);
fprintf(['Weiter:  cmake -S . -B build -DQUAT_IMPL=codegen -DSAFETY_IMPL=codegen ' ...
         '-DCODEGEN_ROOT=%s\n'], fullfile(sitlRoot,'codegen'));
