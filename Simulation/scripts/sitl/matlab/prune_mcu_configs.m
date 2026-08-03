function prune_mcu_configs(mdl)
%prune_mcu_configs  Config-Set-Wildwuchs auf mcu/mcu_flat aufraeumen (Modell-Hygiene).
%   Entfernt nur die nummerierten Duplikate 'ert_cpp_sitl<N>' / 'ert_cpp_arm<N>'
%   (entstehen, wenn configure_mcu_codegen bei Namenskollision wiederholt laeuft)
%   und bei mcu_flat zusaetzlich 'ert_cpp_sitl_flat<N>' / 'ert_cpp_arm_flat<N>'.
%   Die kanonischen Namen ohne Nummer sowie jeder andere Config (z.B. der
%   Modell-Default) bleiben unangetastet. Ein Straggler faellt nur weg, wenn der
%   zugehoerige kanonische Name existiert, damit nie der letzte verschwindet.
%
%   Voraussetzung: das Modell darf nicht interaktiv offen sein (das Skript speichert).
%   Aufruf z.B.:  openProject('DROMA.prj'); prune_mcu_configs('mcu_flat')
if nargin < 1, mdl = 'mcu'; end
load_system(mdl);

names   = getConfigSets(mdl);
haveCan = @(base) any(strcmp(names, base));
isStrag = @(n,base) ~isempty(regexp(n, ['^' base '\d+$'], 'once'));

% Kanonische Basen. Bei mcu_flat stehen die _flat-Varianten vorn: sie sind die
% richtigen fuer dieses Modell, die unbenannten sind von mcu geerbter Ballast.
% Reihenfolge = Vorrang beim Aktiv-Setzen; sitl vor arm (Gate-B-zertifizierter Stand).
bases = {'ert_cpp_sitl','ert_cpp_arm'};
if endsWith(mdl,'_flat')
    bases = [{'ert_cpp_sitl_flat','ert_cpp_arm_flat'}, bases];
end

% Aktiven Config auf einen Keeper setzen (Detach eines aktiven Configs ist verboten).
prefer = '';
for i = 1:numel(bases)
    if haveCan(bases{i}), prefer = bases{i}; break; end
end
if ~isempty(prefer) && ~strcmp(getActiveConfigSet(mdl).Name, prefer)
    setActiveConfigSet(mdl, prefer);
end

removed = {};
for i = 1:numel(names)
    n = names{i};
    for b = 1:numel(bases)
        if isStrag(n, bases{b}) && haveCan(bases{b})
            detachConfigSet(mdl, n);
            removed{end+1} = n; %#ok<AGROW>
            break
        end
    end
end

fprintf('Entfernt (%d): %s\n', numel(removed), strjoin(removed, ', '));
fprintf('Behalten: %s\n', strjoin(getConfigSets(mdl), ', '));
fprintf('Aktiv:    %s\n', getActiveConfigSet(mdl).Name);
if isempty(removed)
    fprintf('Nichts zu tun (keine nummerierten Stragglers).\n');
else
    save_system(mdl);
    fprintf('%s.slx gespeichert.\n', mdl);
end
end
