% Audio-Datei einlesen
[audio_data, Fs] = audioread('motor_aufnahme.wav');

% Falls die Aufnahme in Stereo ist, auf einen Kanal (Mono) reduzieren
if size(audio_data, 2) > 1
    audio_data = audio_data(:, 1);
end

% Spektrum berechnen und plotten
figure;
pspectrum(audio_data, Fs, 'FrequencyLimits', [0 2000]); % Fokus auf 0-2000 Hz
title('Frequenzspektrum der Quadrokopter-Motoren');
xlabel('Frequenz (Hz)');
ylabel('Leistung (dB)');
grid on;