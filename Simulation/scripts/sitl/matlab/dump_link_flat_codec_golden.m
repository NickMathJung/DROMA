%% dump_link_flat_codec_golden.m — Golden fuer den FLATNESS-Codec-Cross-Check.
%  Pendant zu dump_link_codec_golden.m. Erzeugt eine breite CSV: pro Zeile ein
%  Bus_Cmd_flat, durch die massgebliche MATLAB-Kette link_tx_flat -> link_rx_flat
%  gejagt (= die Bloecke in link_flat.slx). Der Host-Test test_link_flat_codec
%  vergleicht dagegen den C++-Codec pktf::pack/unpack_a/unpack_b
%  (mcu_flat_packet.hpp, 2-Frame-OTA).
%
%  Zwei Vergleichsebenen (siehe test_link_flat_codec.cpp):
%    L1 (Wire):   tx_i16[21], tx_q, flags  bit-exakt gegen pktf::pack.
%    L2 (decode): rx_* : Vektoren bit-exakt, q_ext tol 1e-12.
%
%  pdrop = 0 isoliert den Codec (kein Bernoulli-Drop / ZOH).
%
%  int16-Reihenfolge im tx_i16 (aus link_tx_flat):
%    [mocap_pos(3) | p_ref(3) | v_ref(3) | a_ref(3) | j_ref(3) | s_ref(3) | yaw_ref(3)]

here        = fileparts(mfilename('fullpath'));
scriptsRoot = fullfile(here, '..', '..');            % .../scripts
addpath(scriptsRoot, ...
        fullfile(scriptsRoot,'functions'), ...
        fullfile(scriptsRoot,'init'), ...
        fullfile(scriptsRoot,'flatness'));

Ts_inner         = 1e-3;
quadcop          = init_quadcop();
link_flat_params = init_link_flat(quadcop, Ts_inner);
link_flat_params.pdrop = 0;                          % Codec isolieren
clear link_tx_flat                                   % persistente ZOH-States leeren

qI = [1 0 0 0];
Z3 = [0 0 0];

%% --- Testfaelle sammeln ------------------------------------------------------
C = struct('id',{},'moc',{},'qe',{},'p',{},'v',{},'a',{},'j',{},'s',{},'yaw',{},'estop',{},'ack',{});

% -- 1) Hover / Nominal --
C(end+1) = mkf('hover',     [0 0 1], qI, [0 0 1], Z3, Z3, Z3, Z3, Z3, 0, false);
C(end+1) = mkf('hover_ack', [0 0 1], qI, [0 0 1], Z3, Z3, Z3, Z3, Z3, 0, true);

% -- 2) sm3 imax-Branches fuer q_ext --
ex = 1e-3;
C(end+1) = mkf('imax_w', [0 0 1], [1 ex ex ex], [0 0 1], Z3,Z3,Z3,Z3, Z3, 0, false);
C(end+1) = mkf('imax_x', [0 0 1], [ex 1 ex ex], [0 0 1], Z3,Z3,Z3,Z3, Z3, 0, false);
C(end+1) = mkf('imax_y', [0 0 1], [ex ex 1 ex], [0 0 1], Z3,Z3,Z3,Z3, Z3, 0, false);
C(end+1) = mkf('imax_z', [0 0 1], [ex ex ex 1], [0 0 1], Z3,Z3,Z3,Z3, Z3, 0, false);
C(end+1) = mkf('signflip_w', [0 0 1], [-1 ex -ex ex], [0 0 1], Z3,Z3,Z3,Z3, Z3, 0, false);
r2 = 1/sqrt(2);
C(end+1) = mkf('near_half', [0 0 1], [r2 r2 0 0], [0 0 1], Z3,Z3,Z3,Z3, Z3, 0, false);

% -- 3) Mocap-Dropout: q_ext = 0 -> reserviertes Codewort 0 --
C(end+1) = mkf('mocap_invalid', [0 0 1], [0 0 0 0], [0 0 1], Z3,Z3,Z3,Z3, Z3, 0, false);

% -- 4) int16-Saettigung je Feld (jenseits fs=[20,20,20,50,200,2000,(4,20,200)]) --
C(end+1) = mkf('sat_moc', [30 -30 25], qI, [0 0 1], Z3,Z3,Z3,Z3, Z3, 0, false);
C(end+1) = mkf('sat_p',   [0 0 1], qI, [30 -30 25], Z3,Z3,Z3,Z3, Z3, 0, false);
C(end+1) = mkf('sat_v',   [0 0 1], qI, [0 0 1], [30 -30 25], Z3,Z3,Z3, Z3, 0, false);
C(end+1) = mkf('sat_a',   [0 0 1], qI, [0 0 1], Z3, [80 -80 60], Z3,Z3, Z3, 0, false);
C(end+1) = mkf('sat_j',   [0 0 1], qI, [0 0 1], Z3,Z3, [300 -300 250], Z3, Z3, 0, false);
C(end+1) = mkf('sat_s',   [0 0 1], qI, [0 0 1], Z3,Z3,Z3, [3000 -3000 2500], Z3, 0, false);
C(end+1) = mkf('sat_yaw', [0 0 1], qI, [0 0 1], Z3,Z3,Z3,Z3, [8 -40 400], 0, false);

% -- 5) estop-Stufen + ack --
C(end+1) = mkf('estop1',    [0 0 1], qI, [0 0 1], Z3,Z3,Z3,Z3, Z3, 1, false);
C(end+1) = mkf('estop2',    [0 0 1], qI, [0 0 1], Z3,Z3,Z3,Z3, Z3, 2, true);
C(end+1) = mkf('estop2_na', [0 0 1], qI, [0 0 1], Z3,Z3,Z3,Z3, Z3, 2, false);

% -- 6) Zufall: unit-Quats + Werte im/leicht ueber Bereich --
rng(24680,'twister');
Nrand = 200;
for i = 1:Nrand
    qe  = randn(1,4); qe = qe/norm(qe);
    moc = 22*(2*rand(1,3)-1);        % bis knapp ueber fs=20
    p   = 22*(2*rand(1,3)-1);
    v   = 22*(2*rand(1,3)-1);
    a   = 55*(2*rand(1,3)-1);
    j   = 220*(2*rand(1,3)-1);
    s   = 2200*(2*rand(1,3)-1);
    yaw = [4.4*(2*rand-1), 22*(2*rand-1), 220*(2*rand-1)];
    C(end+1) = mkf(sprintf('rand%03d',i), moc, qe, p, v, a, j, s, yaw, mod(i,3), logical(mod(i,2))); %#ok<SAGROW>
end

%% --- durch link_tx_flat -> link_rx_flat jagen --------------------------------
rows = cell(numel(C),1);
for i = 1:numel(C)
    c = C(i);
    cmd_in = struct('mocap_pos',c.moc(:), 'q_ext',c.qe(:), ...
                    'p_ref',c.p(:), 'v_ref',c.v(:), 'a_ref',c.a(:), ...
                    'j_ref',c.j(:), 's_ref',c.s(:), 'yaw_ref',c.yaw(:), ...
                    'estop',uint8(c.estop), 'ack',logical(c.ack));

    [pkt_i16, pkt_q, flags] = link_tx_flat(cmd_in, link_flat_params);  % pdrop=0
    rx = link_rx_flat(pkt_i16, pkt_q, flags, link_flat_params);

    nums = [ c.moc, c.qe, c.p, c.v, c.a, c.j, c.s, c.yaw, double(c.estop), double(c.ack), ...
             double(pkt_i16(:).'), double(pkt_q(1)), double(flags(:).'), ...
             rx.mocap_pos(:).', rx.q_ext(:).', rx.p_ref(:).', rx.v_ref(:).', ...
             rx.a_ref(:).', rx.j_ref(:).', rx.s_ref(:).', rx.yaw_ref(:).', ...
             double(rx.estop), double(rx.ack) ];
    rows{i} = [{c.id}, num2cell(nums)];
end

%% --- CSV schreiben -----------------------------------------------------------
outCsv = fullfile(here, '..', 'data', 'link_flat_codec_golden.csv');
if ~isfolder(fileparts(outCsv)); mkdir(fileparts(outCsv)); end

hdr = [ {'id'}, ...
    strc('in_moc',3), strc('in_qe',4), strc('in_p',3), strc('in_v',3), strc('in_a',3), ...
    strc('in_j',3), strc('in_s',3), strc('in_yaw',3), strc('in_estop'), strc('in_ack'), ...
    strc('tx_i16',21), strc('tx_q'), strc('tx_flags',2), ...
    strc('rx_moc',3), strc('rx_qe',4), strc('rx_p',3), strc('rx_v',3), strc('rx_a',3), ...
    strc('rx_j',3), strc('rx_s',3), strc('rx_yaw',3), strc('rx_estop'), strc('rx_ack') ];

fid = fopen(outCsv,'w'); assert(fid>0,'CSV nicht schreibbar: %s',outCsv);
fprintf(fid,'%s', hdr{1}); fprintf(fid,',%s', hdr{2:end}); fprintf(fid,'\n');
for i = 1:numel(rows)
    r = rows{i};
    fprintf(fid,'%s', r{1});
    for k = 2:numel(r); fprintf(fid,',%.17g', r{k}); end
    fprintf(fid,'\n');
end
fclose(fid);
fprintf('link_flat_codec_golden geschrieben: %s  (%d Zeilen, %d Datenspalten)\n', ...
        outCsv, numel(rows), numel(hdr)-1);

%% --- lokale Helfer -----------------------------------------------------------
function s = mkf(id,moc,qe,p,v,a,j,sn,yaw,estop,ack)
    s = struct('id',id,'moc',moc(:).','qe',qe(:).','p',p(:).','v',v(:).', ...
               'a',a(:).','j',j(:).','s',sn(:).','yaw',yaw(:).', ...
               'estop',estop,'ack',ack);
end

function c = strc(base, n)
    if nargin < 2 || n == 1
        c = {base};
    else
        c = arrayfun(@(k) sprintf('%s%d',base,k), 1:n, 'UniformOutput', false);
    end
end
