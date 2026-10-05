function flightTrajectory(id)
%   flightTrajectory Animierte 3D-Trajektorie einer Drohne anhand der ID
%   Lädt Daten aus dem Basis-Workspace und vergleicht Flugbahn mit Referenz.
arguments
    id (1,1) double = 1
end

% Daten aus dem Basis-Workspace laden (analog zu flight_evaluation)
out = evalin('base', 'out');
ids = evalin('base', 'mocap.streaming_ids');
d = find(ids == id, 1);
assert(~isempty(d), 'flightTrajectory: id=%d nicht in mocap.streaming_ids %s.', ...
    id, mat2str(ids));

nt = numel(out.tout);
g = @(name) orient_ts(out.get(sprintf('%s_%d', name, d)).Data, nt);

%  Variablen extrahieren
x     = g('mocap_pos');
x_ref = g('x_ref');

%  Plot-Fenster vorbereiten
figure('Name', sprintf('Live-Flug Drohne id=%d', id));

% Achsenlimits berechnen (inklusive Referenz und echten Daten, damit nichts springt)
min_x = min([x(:,1); x_ref(:,1)]); max_x = max([x(:,1); x_ref(:,1)]);
min_y = min([x(:,2); x_ref(:,2)]); max_y = max([x(:,2); x_ref(:,2)]);
min_z = min([x(:,3); x_ref(:,3)]); max_z = max([x(:,3); x_ref(:,3)]);

% Puffer hinzufügen, damit die Linien nicht am Rand kleben
xlim([min_x, max_x]);
ylim([min_y, max_y]);
zlim([min_z, max_z]);

grid on;
view(3); % 3D Ansicht
hold on;
xlabel('X-Achse [m]');
ylabel('Y-Achse [m]');
zlabel('Z-Achse [m]');
title(sprintf('Live 3D-Trajektorie (Drohne %d)', id));

%  Die Referenz-Trajektorie vorab als gepunktete Linie zeichnen
plot3(x_ref(:,1), x_ref(:,2), x_ref(:,3), 'k:', 'LineWidth', 1.5, 'DisplayName', 'Referenz');

% Animierte Linie für den echten Flug vorbereiten
curve = animatedline('Color', 'b', 'LineWidth', 2, 'DisplayName', 'Flugbahn');
legend('show', 'Location', 'best');

% Startpunkt als roten Punkt markieren
plot3(x(1,1), x(1,2), x(1,3), 'ro', 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');

% Berechne den durchschnittlichen Zeitabstand dt
t_flight = out.tout;
dt = median(diff(t_flight)); % z.B. 0.01 Sekunden
numPoints = size(x, 1);
for k = 1:numPoints
    addpoints(curve, x(k,1), x(k,2), x(k,3));
    
    % Wir überspringen ein paar Frames, damit MATLAB nicht überlastet wird,
    % multiplizieren aber die Pause mit der Anzahl der übersprungenen Frames (z.B. 5)
    if mod(k, 5) == 0
        pause(dt * 5); 
    end
end

% drawnow; % Letzten Frame sicherheitshalber zeichnen

end

function A = orient_ts(A, nt)
% Timeseries-Daten auf [nt x k] orientieren.
A = squeeze(A);
if size(A,1) ~= nt && size(A,2) == nt
    A = A.';
end
end