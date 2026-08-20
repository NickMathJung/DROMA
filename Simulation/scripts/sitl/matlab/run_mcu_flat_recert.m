function run_mcu_flat_recert(proj_root)
% run_mcu_flat_recert  --  mcu_flat.slx neu generieren + Golden neu aufzeichnen.
%   Laeuft headless.
%   proj_root = Simulation-Wurzel (enthaelt DROMA.prj, scripts\, models\).
%   throttle_poly.hpp wird nicht neu gedumpt.
sitl = fullfile(proj_root,'scripts','sitl');

fprintf('== openProject ==\n');
openProject(fullfile(proj_root,'DROMA.prj'));
load_system('quadcop');              % legt Ts_inner/quadcop im Base-Workspace an
assert(evalin('base','exist(''Ts_inner'',''var'')'), 'Ts_inner fehlt (PreLoadFcn?).');

oldcd = cd(sitl);
cleanup = onCleanup(@() cd(oldcd));

% CodeGen-/Cache-Ordner auf scripts\sitl umlenken, danach auf proj_root
Simulink.fileGenControl('set','CodeGenFolder',sitl,'CacheFolder',sitl,'createDir',true);
restoreFolders = onCleanup(@() Simulink.fileGenControl('set', ...
    'CodeGenFolder',proj_root,'CacheFolder',proj_root,'createDir',true));

fprintf('== configure_mcu_flat_codegen + slbuild ==\n');
clear configure_mcu_flat_codegen
configure_mcu_flat_codegen('mcu_flat');
slbuild('mcu_flat');

hdr = fileread(fullfile(sitl,'mcu_flat_ert_rtw','mcu_flat.h'));
assert(contains(hdr,'MCU_FLAT'), 'mcu_flat.h ohne Klasse MCU_FLAT — Regen fehlgeschlagen?');
assert(contains(hdr,'Bus_Cmd_flat'), 'mcu_flat.h ohne Bus_Cmd_flat — falsches Modell?');
fprintf('OK: mcu_flat.h enthaelt MCU_FLAT + Bus_Cmd_flat.\n');

fprintf('== log_mcu_flat_golden ==\n');
clear log_mcu_flat_golden
run(fullfile(sitl,'matlab','log_mcu_flat_golden.m'));

fprintf('\n== Fertig: Regen und Golden. Naechster Schritt: Gate B (ctest -R McuFlat). ==\n');
end
