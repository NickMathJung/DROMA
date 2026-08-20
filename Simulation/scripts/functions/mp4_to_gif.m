function gif_file = mp4_to_gif(mp4_file, t_start, t_end, fps_out, scale)
%mp4_to_gif  Ausschnitt eines MP4 als GIF gleichen Namens speichern.
%   mp4_to_gif('data\videos\swarm_1_2_3.mp4', 2, 12)
%   Schneidet [t_start, t_end] aus und schreibt <name>.gif daneben.
%   fps_out (Default 10) und scale (Default 0.5, Pixel-Dezimation) halten die
%   GIF-Groesse im Rahmen.
arguments
    mp4_file char
    t_start (1,1) double = 0
    t_end   (1,1) double = inf
    fps_out (1,1) double = 10
    scale   (1,1) double = 0.5
end
v = VideoReader(mp4_file);
t_end = min(t_end, v.Duration);
assert(t_start >= 0 && t_start < t_end, 'mp4_to_gif: ungueltiger Zeitbereich.');
[p, n] = fileparts(mp4_file);
gif_file = fullfile(p, [n '.gif']);
ds = max(1, round(1 / scale));   % Pixel-Dezimation statt imresize (toolboxfrei)

first = true;
for tq = t_start : 1/fps_out : t_end
    v.CurrentTime = tq;
    if ~hasFrame(v), break; end
    fr = readFrame(v);
    fr = fr(1:ds:end, 1:ds:end, :);
    [A, map] = rgb2ind(fr, 256);
    if first
        imwrite(A, map, gif_file, 'gif', 'LoopCount', inf, 'DelayTime', 1/fps_out);
        first = false;
    else
        imwrite(A, map, gif_file, 'gif', 'WriteMode', 'append', 'DelayTime', 1/fps_out);
    end
end
d = dir(gif_file);
fprintf('[mp4_to_gif] %s: %.1f..%.1f s, %g fps, 1/%d Pixel -> %.1f MB\n', ...
    gif_file, t_start, t_end, fps_out, ds, d.bytes/1e6);
end
