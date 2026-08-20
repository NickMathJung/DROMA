function setup_motive_path()
%setup_motive_path  Pfade fuer die Motive/NatNet-Anbindung setzen.
%   Einmal pro MATLAB-Session aufrufen.
%
%   Legt auf den Pfad:
%     scripts\motive\ -> MotiveMocap
%     ...\Motive\OptiTrack_MATLAB_Plugin_1.1.0\Matlab\ -> natnet, quaternion
%
%   Das Plugin liegt ausserhalb des Repos unter DROMA\Motive\.
%   Legt ausserdem <plugin>\Matlab\assemblypath.txt mit dem DLL-Pfad an.

    here   = fileparts(mfilename('fullpath')); % ...\scripts\motive
    sim    = fileparts(fileparts(here)); % ...\Simulation
    droma  = fileparts(sim); % ...\DROMA
    plugin = fullfile(droma,'Motive','OptiTrack_MATLAB_Plugin_1.1.0');
    pmat   = fullfile(plugin,'Matlab');
    dll    = fullfile(plugin,'NatNetML.dll');

    addpath(here);
    if ~isfolder(pmat)
        error('setup_motive_path:plugin', ...
              ['OptiTrack-Plugin nicht gefunden: %s\n' ...
               'ZIP nach %s entpacken.'], pmat, plugin);
    end
    addpath(pmat);

    if ~isfile(dll)
        error('setup_motive_path:dll', 'NatNetML.dll fehlt: %s', dll);
    end
    apf = fullfile(pmat,'assemblypath.txt');
    if ~isfile(apf) || ~strcmp(strtrim(fileread(apf)), dll)
        fid = fopen(apf,'w');  assert(fid>0, 'assemblypath.txt nicht schreibbar');
        fprintf(fid,'%s',dll); fclose(fid);
        fprintf('setup_motive_path: assemblypath.txt geschrieben -> %s\n', dll);
    end

    fprintf('setup_motive_path: OK\n');
    fprintf('  MotiveMocap : %s\n', which('MotiveMocap'));
    fprintf('  natnet      : %s\n', which('natnet'));
end
