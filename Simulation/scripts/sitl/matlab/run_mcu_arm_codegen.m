function run_mcu_arm_codegen(proj_root)
% run_mcu_arm_codegen — mcu.slx fuer Teensy 4.1 / Cortex-M7 generieren.
%   Nur GenCodeOnly; kompiliert wird spaeter ueber die Teensy/PlatformIO-Toolchain,
%   nicht aus MATLAB. Output nach hardware\mcu_arm\ (eigener CodeGen-/Cache-Ordner),
%   damit das SITL-zertifizierte scripts\sitl\mcu_ert_rtw\ (x86, Gate B) unberuehrt
%   bleibt. Als Funktion geschrieben, damit die lokalen Vars den 'clear' in params.m
%   (quadcop-PreLoadFcn) ueberleben.
armdir = fullfile(proj_root,'hardware','mcu_arm');

openProject(fullfile(proj_root,'DROMA.prj'));
load_system('quadcop');              % PreLoadFcn -> params.m -> Ts_inner/quadcop
assert(evalin('base','exist(''Ts_inner'',''var'')'), 'Ts_inner fehlt (PreLoadFcn?).');

if ~isfolder(armdir), mkdir(armdir); end

% --- CodeGen-/Cache-Ordner auf hardware\mcu_arm umlenken (und wiederherstellen) --
cfg = Simulink.fileGenControl('getConfig');
oldCGF = cfg.CodeGenFolder; oldCF = cfg.CacheFolder;
Simulink.fileGenControl('set','CodeGenFolder',armdir,'CacheFolder',armdir,'createDir',true);
restoreFolders = onCleanup(@() Simulink.fileGenControl('set', ...
    'CodeGenFolder',oldCGF,'CacheFolder',oldCF));

% --- ARM-Config aktiv setzen + generieren ---
clear configure_mcu_codegen
configure_mcu_codegen('mcu','arm');
slbuild('mcu');

genroot = fullfile(armdir,'mcu_ert_rtw');
assert(isfolder(genroot), 'ARM-Codegen-Ordner fehlt: %s', genroot);
hdr = fileread(fullfile(genroot,'mcu.h'));
assert(contains(hdr,'throttle'), 'ARM-mcu.h ohne throttle — falsches Modell?');
fprintf('\n== ARM-Codegen fertig -> %s ==\n', genroot);
fprintf('   Verifikation (x86-Intrinsics, Wortbreiten, Klasse MCU) per grep im Host.\n');
end
