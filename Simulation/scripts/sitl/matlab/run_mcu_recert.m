function run_mcu_recert(proj_root)
% run_mcu_recert — mcu.slx (throttle-Outport) neu generieren, throttle-Polynom
% fuer die Host-Invariante dumpen, Golden neu aufzeichnen. Laeuft headless.
%   proj_root = Simulation-Wurzel (enthaelt DROMA.prj, scripts\, models\).
sitl = fullfile(proj_root,'scripts','sitl');

fprintf('== openProject ==\n');
openProject(fullfile(proj_root,'DROMA.prj'));
load_system('quadcop');              % PreLoadFcn -> params.m -> Ts_inner/quadcop in base
assert(evalin('base','exist(''Ts_inner'',''var'')'), 'Ts_inner fehlt (PreLoadFcn?).');
quadcop = evalin('base','quadcop');

oldcd = cd(sitl);                    % slbuild-Ausgabe -> scripts\sitl\mcu_ert_rtw
cleanup = onCleanup(@() cd(oldcd));

% Ausgabeordner EXPLIZIT setzen (gleicher Fix wie run_mcu_flat_recert: das
% Projekt pinnt CodeGenFolder auf die Wurzel, sonst bricht slbuild ab, sobald
% in derselben Sitzung vorher ein anderer Codegen-Lauf lief).
Simulink.fileGenControl('set','CodeGenFolder',sitl,'CacheFolder',sitl,'createDir',true);
restoreFolders = onCleanup(@() Simulink.fileGenControl('set', ...
    'CodeGenFolder',proj_root,'CacheFolder',proj_root,'createDir',true)); %#ok<NASGU>

fprintf('== configure_mcu_codegen + slbuild ==\n');
clear configure_mcu_codegen
configure_mcu_codegen('mcu');
slbuild('mcu');

% --- ExtY-Kontrolle: throttle jetzt vorhanden? --------------------------------
hdr = fileread(fullfile(sitl,'mcu_ert_rtw','mcu.h'));
assert(contains(hdr,'throttle'), 'mcu.h enthaelt kein throttle — Regen fehlgeschlagen?');
fprintf('OK: mcu.h enthaelt throttle.\n');

% --- throttle-Kennlinie fuer die Host-Invariante dumpen -----------------------
p = quadcop.p_from_omega(:).';       % [p1 p2 0], Fit ueber omega, durch den Ursprung
hpp = fullfile(sitl,'include','throttle_poly.hpp');
fid = fopen(hpp,'w'); assert(fid>0,'throttle_poly.hpp nicht schreibbar');
fprintf(fid,'// Generiert von run_mcu_recert.m aus quadcop.p_from_omega — bitte nicht editieren.\n');
fprintf(fid,'// throttle = clamp(polyval(P_THROTTLE, rotor_cmd) * U_DS / Vc, 0, 100)\n');
fprintf(fid,'//   mit Vc = clamp(V_lp, V_THR_MIN, V_THR_MAX); V_lp = Tiefpass (safety.tau_thr)\n');
fprintf(fid,'//   ueber batt_k*batt_count + batt_b (schneller V-Pfad, 05.08.2026).\n');
fprintf(fid,'// Eingang ist rotor_cmd (= omega), NICHT omega^2 — das Polynom wurde auf\n');
fprintf(fid,'// omega umgestellt (Fit durch den Ursprung, Residuum 0.8%% statt 5.6%%).\n');
fprintf(fid,'#ifndef THROTTLE_POLY_HPP\n#define THROTTLE_POLY_HPP\nnamespace mcuref {\n');
fprintf(fid,'static constexpr int    P_THROTTLE_N   = %d;\n', numel(p));
fprintf(fid,'static constexpr double P_THROTTLE[%d] = { ', numel(p));
fprintf(fid,'%.17g', p(1)); fprintf(fid,', %.17g', p(2:end)); fprintf(fid,' };\n');
fprintf(fid,'static constexpr double U_DS       = %.17g;\n', quadcop.U_ds);
fprintf(fid,'static constexpr double V_THR_MIN  = %.17g;\n', quadcop.V_thr_min);
fprintf(fid,'static constexpr double V_THR_MAX  = %.17g;\n', quadcop.V_thr_max);
fprintf(fid,'static constexpr double THROTTLE_MIN = 0.0, THROTTLE_MAX = 100.0;\n');
fprintf(fid,'}  // namespace mcuref\n#endif\n');
fclose(fid);
fprintf('throttle_poly.hpp geschrieben: p = [%.17g %.17g %.17g], U_ds=%.4g, Vclamp=[%.4g %.4g]\n', ...
        p(1),p(2),p(3), quadcop.U_ds, quadcop.V_thr_min, quadcop.V_thr_max);

% --- Golden neu (mit throttle-Spalten) ----------------------------------------
fprintf('== log_mcu_golden ==\n');
clear log_mcu_golden
run(fullfile(sitl,'matlab','log_mcu_golden.m'));

fprintf('\n== Fertig: Regen und Golden. Naechster Schritt: Gate B (ctest). ==\n');
end
