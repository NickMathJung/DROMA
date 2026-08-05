function run_mcu_flat_arm_codegen(proj_root)
% run_mcu_flat_arm_codegen — mcu_flat.slx fuer Teensy 4.1 / Cortex-M7 generieren.
%   Pendant zu run_mcu_arm_codegen (Kaskade). Nur GenCodeOnly — kompiliert wird
%   spaeter ueber die Teensy-Toolchain. Output nach hardware\mcu_flat_arm\
%   (eigener CodeGen-/Cache-Ordner), damit das SITL-zertifizierte
%   scripts\sitl\mcu_flat_ert_rtw\ unberuehrt bleibt.
armdir = fullfile(proj_root,'hardware','mcu_flat_arm');

openProject(fullfile(proj_root,'DROMA.prj'));
load_system('quadcop');
assert(evalin('base','exist(''Ts_inner'',''var'')'), 'Ts_inner fehlt (PreLoadFcn?).');

if ~isfolder(armdir)
    mkdir(armdir);
end

% --- CodeGen-/Cache-Ordner auf hardware\mcu_flat_arm umlenken (und zuruecksetzen)
% Zurueck geht es auf proj_root, NICHT auf den vorgefundenen Wert: das Projekt pinnt
% beides auf die Wurzel, und ein "merken und zurueckschreiben" verewigt den
% temporaeren Wert, sobald ein Lauf ihn einmal hinterlassen hat. Symptom danach:
% jede Simulation bricht ab mit "Current working folder contains simulation
% artifacts that can shadow artifacts in the folder ... CacheFolder".
Simulink.fileGenControl('set','CodeGenFolder',armdir,'CacheFolder',armdir,'createDir',true);
restoreFolders = onCleanup(@() Simulink.fileGenControl('set', ...
    'CodeGenFolder',proj_root,'CacheFolder',proj_root,'createDir',true));

% --- ARM-Config aktiv setzen + generieren ---
clear configure_mcu_flat_codegen
configure_mcu_flat_codegen('mcu_flat','arm');
slbuild('mcu_flat');

genroot = fullfile(armdir,'mcu_flat_ert_rtw');
assert(isfolder(genroot), 'ARM-Codegen-Ordner fehlt: %s', genroot);
hdr = fileread(fullfile(genroot,'mcu_flat.h'));
assert(contains(hdr,'MCU_FLAT'), 'ARM-mcu_flat.h ohne Klasse MCU_FLAT — falsches Modell?');
assert(contains(hdr,'Bus_Cmd_flat'), 'ARM-mcu_flat.h ohne Bus_Cmd_flat — falsches Modell?');
src = fileread(fullfile(genroot,'mcu_flat.cpp'));
assert(~contains(src,'emmintrin'), 'ARM-mcu_flat.cpp enthaelt x86-SSE2-Intrinsics!');
fprintf('\n== ARM-Codegen fertig -> %s ==\n', genroot);
fprintf('   Verifikation (keine x86-Intrinsics, Klasse MCU_FLAT, Bus_Cmd_flat) OK.\n');
end
