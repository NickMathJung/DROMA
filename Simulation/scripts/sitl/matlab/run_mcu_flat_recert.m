function run_mcu_flat_recert(proj_root)
% run_mcu_flat_recert — mcu_flat.slx neu generieren + Golden neu aufzeichnen.
%   Pendant zu run_mcu_recert (Kaskade). Laeuft headless.
%   proj_root = Simulation-Wurzel (enthaelt DROMA.prj, scripts\, models\).
%   throttle_poly.hpp wird NICHT neu gedumpt (gehoert der Kaskaden-Recert;
%   Kennlinie ist zwischen beiden Varianten geteilt).
sitl = fullfile(proj_root,'scripts','sitl');

fprintf('== openProject ==\n');
openProject(fullfile(proj_root,'DROMA.prj'));
load_system('quadcop');              % PreLoadFcn -> params.m -> Ts_inner/quadcop in base
assert(evalin('base','exist(''Ts_inner'',''var'')'), 'Ts_inner fehlt (PreLoadFcn?).');

oldcd = cd(sitl);                    % slbuild-Ausgabe -> scripts\sitl\mcu_flat_ert_rtw
cleanup = onCleanup(@() cd(oldcd));

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
