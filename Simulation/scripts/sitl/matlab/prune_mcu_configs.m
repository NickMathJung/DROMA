function prune_mcu_configs(mdl)
%prune_mcu_configs  Nummerierte Config-Set-Duplikate auf mcu/mcu_flat entfernen.
%   Entfernt 'ert_cpp_sitl<N>' / 'ert_cpp_arm<N>' und bei mcu_flat zusaetzlich
%   'ert_cpp_sitl_flat<N>' / 'ert_cpp_arm_flat<N>'. Die kanonischen Namen ohne
%   Nummer und jeder andere Config bleiben unangetastet; ein Duplikat faellt nur
%   weg, wenn der zugehoerige kanonische Name existiert.
%
%   Das Modell darf nicht interaktiv offen sein, das Skript speichert.
%   Aufruf z.B.:  openProject('DROMA.prj'); prune_mcu_configs('mcu_flat')
if nargin < 1, mdl = 'mcu'; end
load_system(mdl);

names   = getConfigSets(mdl);
haveCan = @(base) any(strcmp(names, base));
isStrag = @(n,base) ~isempty(regexp(n, ['^' base '\d+$'], 'once'));

% Kanonische Basen, Reihenfolge = Vorrang beim Aktiv-Setzen.
bases = {'ert_cpp_sitl','ert_cpp_arm'};
if endsWith(mdl,'_flat')
    bases = [{'ert_cpp_sitl_flat','ert_cpp_arm_flat'}, bases];
end

% Aktiven Config auf einen Keeper setzen.
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
