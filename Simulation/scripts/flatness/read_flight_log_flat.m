function log = read_flight_log_flat(fname)
%read_flight_log_flat  Blackbox-Datei FLATnnn.BIN von der Drohnen-SD einlesen.
%
%   log = read_flight_log_flat('F:\FLAT000.BIN')
%
%   Rueckgabe (t jeweils in s, Nullpunkt = erster Fast-Record):
%     .hdr        Kopfdaten (id, Betriebsart, Skalen, Tickbereich)
%     .t_fast     Nx1   1-kHz-Zeitachse
%     .gyro       Nx3   rad/s, biasfrei, Body-Frame
%     .acc        Nx3   m/s^2, Body-Frame
%     .t_slow     Mx1   100-Hz-Zeitachse
%     .mocap_pos  Mx3 | .p_ref Mx3 | .q_ext Mx4 | .thr Mx4 [%] | .batt_count Mx1
%     .estop .led .ack .flags   Mx1 uint8
%                 flags: Bit0 Link-Timeout, Bit1 Puffer voll (25 s erreicht),
%                        Bit2 Aufzeichnung beendet (estop/Landung/voll)
%     .k_hat      Mx1   Schub-Skalenschaetzung
%     .F          Mx1   kommandierter Schub [N]
%     .aint       Mx3   Integratorbeitrag
%     .ufb        Mx3   Rueckfuehrung vor der Klemme [m/s^4]
%     .sat        Mx3   logical, ufb ueber UFB_MAX
%     .w          Mx1   Adaptions-/Integrations-Freigabe in [0,1]
arguments (Input)
    fname (1,:) char
end

fid = fopen(fname,'r');
assert(fid > 0, 'Datei nicht lesbar: %s', fname);
raw = fread(fid, inf, '*uint8'); fclose(fid);
assert(numel(raw) >= 64, 'Datei zu kurz (%d B) — kein gueltiger Header.', numel(raw));

u16 = @(o) double(typecast(raw(o+1:o+2),'uint16'));
u32 = @(o) double(typecast(raw(o+1:o+4),'uint32'));
f32 = @(o) double(typecast(raw(o+1:o+4),'single'));

h.magic      = char(raw(1:7)).';
assert(strcmp(h.magic,'DROMAFL'), 'Magic falsch ("%s") — keine DROMA-Blackbox.', h.magic);
h.version    = u16(8);
assert(h.version == 3, ['Formatversion %d, dieser Leser erwartet 3. ' ...
       'v1 kannte die Reglerzustaende noch nicht, v2 die Adaptions-Freigabe.'], h.version);
h.own_id     = double(raw(11));
h.hal_mode   = double(raw(12));
h.ts_fast_us = u32(12);
h.div_slow   = u16(16);
h.rec_fast   = u16(18);
h.rec_slow   = u16(20);
h.gyro_scale = f32(24);
h.acc_scale  = f32(28);
h.n_fast     = u32(32);
h.n_slow     = u32(36);
h.tick_first = u32(40);
h.tick_last  = u32(44);
h.t_dump_ms  = u32(48);
assert(h.rec_fast == 12 && h.rec_slow == 76, ...
       'Recordlaengen (%d/%d) passen nicht zu diesem Leser.', h.rec_fast, h.rec_slow);

need = 64 + h.n_fast*h.rec_fast + h.n_slow*h.rec_slow;
assert(numel(raw) >= need, 'Datei unvollstaendig: %d B, erwartet %d B (Dump abgebrochen?).', ...
       numel(raw), need);

% ---- Fast-Block: N x 12 B, je 6 int16 ---------------------------------------
o = 64;
F = reshape(typecast(raw(o+1 : o+h.n_fast*12), 'int16'), 6, []).';
log.gyro = double(F(:,1:3)) / h.gyro_scale;
log.acc  = double(F(:,4:6)) / h.acc_scale;
dt = h.ts_fast_us * 1e-6;
log.t_fast = (0:h.n_fast-1).' * dt;

% ---- Slow-Block: M x 58 B, gemischte Typen -> spaltenweise herausschneiden ---
o = o + h.n_fast*12;
S = reshape(raw(o+1 : o+h.n_slow*76), 76, []).'; % M x 76 uint8
% Spaltenbereiche je Feld, 1-basiert
t = S(:, 1: 4).'; tick          = double(typecast(t(:),'uint32'));
t = S(:, 5:16).'; log.mocap_pos = reshape(double(typecast(t(:),'single')), 3, []).';
t = S(:,17:28).'; log.p_ref     = reshape(double(typecast(t(:),'single')), 3, []).';
t = S(:,29:44).'; log.q_ext     = reshape(double(typecast(t(:),'single')), 4, []).';
t = S(:,45:52).'; log.thr       = reshape(double(typecast(t(:),'uint16')), 4, []).' / 100; % -> %
t = S(:,53:54).'; log.batt_count= double(typecast(t(:),'uint16'));
log.estop       = S(:,55);
log.led         = S(:,56);
log.ack         = S(:,57);
log.flags       = S(:,58);
% Reglerzustaende, Skalen stehen nicht im Header
KHAT_SCALE = 20000; F_SCALE = 800; AINT_SCALE = 80; UFB_SCALE = 8; W_SCALE = 20000; UFB_MAX = 700;
t = S(:,59:60).'; log.k_hat = double(typecast(t(:),'int16')) / KHAT_SCALE;
t = S(:,61:62).'; log.F     = double(typecast(t(:),'int16')) / F_SCALE;
t = S(:,63:68).'; log.aint  = reshape(double(typecast(t(:),'int16')), 3, []).' / AINT_SCALE;
t = S(:,69:74).'; log.ufb   = reshape(double(typecast(t(:),'int16')), 3, []).' / UFB_SCALE;
t = S(:,75:76).'; log.w     = double(typecast(t(:),'int16')) / W_SCALE;
log.sat         = abs(log.ufb) > UFB_MAX;
log.t_slow      = (tick - double(h.tick_first)) * 1e-3;
log.hdr         = h;

modes = {'BENCH','THRUST','FLIGHT'};
fprintf(['Blackbox %s: id=%d, %s, %.2f s @ %.0f Hz (%d Fast / %d Slow)\n'], ...
        fname, h.own_id, modes{min(h.hal_mode,2)+1}, h.n_fast*dt, 1/dt, h.n_fast, h.n_slow);
if any(bitand(log.flags, 2))
    fprintf('  Hinweis: Puffer war voll (25 s) — die Aufzeichnung endet vor dem Flugende.\n');
end
if any(bitand(log.flags, 1)), fprintf('  Hinweis: Link-Timeout in %d von %d Slow-Records.\n', ...
        sum(bitand(log.flags,1)>0), h.n_slow); end
fprintf('  Adaption/Integration frei (w=1): %.1f %% der Zeit, teilweise (0<w<1): %.1f %%\n', ...
        100*mean(log.w >= 0.999), 100*mean(log.w > 0.001 & log.w < 0.999));
if any(log.sat(:))
    ax = 'xyz';
    for a = 1:3
        if any(log.sat(:,a))
            fprintf('  UFB_MAX-Saettigung in %s: %d von %d Records (%.1f %%), max |u_fb| = %.0f\n', ...
                ax(a), sum(log.sat(:,a)), h.n_slow, 100*sum(log.sat(:,a))/h.n_slow, max(abs(log.ufb(:,a))));
        end
    end
end
end
