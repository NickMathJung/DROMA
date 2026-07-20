%% dump_link_codec_golden.m — Golden fuer den Codec-Cross-Check (Sim == HW).
%  Erzeugt eine breite CSV: pro Zeile ein Bus_Cmd, durch die massgebliche
%  MATLAB-Kette link_tx -> link_rx gejagt (= chart_40/chart_50 in link.slx).
%  Der Host-Test test_link_codec vergleicht dagegen den C++-Codec pkt::pack/
%  pkt::unpack (mcu_packet.hpp).
%
%  Zwei Vergleichsebenen (siehe test_link_codec.cpp):
%    L1 (Wire):   tx_i16[7], tx_q[3], flags  bit-exakt gegen pkt::pack.
%    L2 (decode): rx_* : F/Om/tau bit-exakt, Quats tol 1e-12 gegen pkt::unpack.
%
%  pdrop = 0 isoliert den Codec (kein Bernoulli-Drop / ZOH). Der xorshift/ZOH-
%  Pfad ist Kanalverhalten, nicht Codec, und hat kein C++-Pendant.
%
%  int16-Reihenfolge im tx_i16 (aus link_tx: v=[F; Omega_ref(3); tau_ref(3)]):
%    tx_i16 = [F | Om1 Om2 Om3 | tau1 tau2 tau3].
%  Quat-Reihenfolge tx_q = [q_des ; q_ref ; q_ext], scalar-first [w x y z].

here        = fileparts(mfilename('fullpath'));
scriptsRoot = fullfile(here, '..', '..');            % .../scripts
addpath(scriptsRoot, ...
        fullfile(scriptsRoot,'functions'), ...
        fullfile(scriptsRoot,'init'));

Ts_inner    = 1e-3;                                   % = params.m: 1/(10*100)
quadcop     = init_quadcop();
link_params = init_link(quadcop, Ts_inner);
link_params.pdrop = 0;                               % Codec isolieren
clear link_tx                                        % persistente ZOH-States leeren

qI = [1 0 0 0];                                      % Identitaet
Z3 = [0 0 0];
Fh = quadcop.m*quadcop.g;                            % Hover-Schub [N]

%% --- Testfaelle sammeln (Struct-Array) --------------------------------------
C = struct('id',{},'F',{},'qd',{},'qr',{},'Om',{},'tr',{},'qe',{},'estop',{},'ack',{});

% -- 1) Hover / Nominal --
C(end+1) = mk('hover',     Fh, qI,qI,Z3,Z3, qI, 0, false);
C(end+1) = mk('hover_ack', Fh, qI,qI,Z3,Z3, qI, 0, true);

% -- 2) sm3 imax-Branches (groesste Komponente je Slot) -- qd=qr=qe=SQ --
ex = 1e-3;
C(end+1) = mk('imax_w', Fh, [1 ex ex ex], [1 ex ex ex], Z3, Z3, [1 ex ex ex], 0, false);
C(end+1) = mk('imax_x', Fh, [ex 1 ex ex], [ex 1 ex ex], Z3, Z3, [ex 1 ex ex], 0, false);
C(end+1) = mk('imax_y', Fh, [ex ex 1 ex], [ex ex 1 ex], Z3, Z3, [ex ex 1 ex], 0, false);
C(end+1) = mk('imax_z', Fh, [ex ex ex 1], [ex ex ex 1], Z3, Z3, [ex ex ex 1], 0, false);

% -- 3) Sign-Flip an imax (groesste Komponente negativ; q == -q) --
C(end+1) = mk('signflip_w', Fh, [-1 ex -ex ex], [-1 ex -ex ex], Z3, Z3, [-1 ex -ex ex], 0, false);
C(end+1) = mk('signflip_z', Fh, [ex -ex ex -1], [ex -ex ex -1], Z3, Z3, [ex -ex ex -1], 0, false);

% -- 4) Komponenten nahe +-1/sqrt(2) (Clamp/Round-Grenze bei +-511) --
r2 = 1/sqrt(2);
C(end+1) = mk('near_half_ww', Fh, [r2 r2 0 0], [r2 r2 0 0], Z3, Z3, [r2 r2 0 0], 0, false);
C(end+1) = mk('near_half_pp', Fh, [0.7072 0.7070 1e-4 1e-4], [0.7072 0.7070 1e-4 1e-4], Z3, Z3, [0.7072 0.7070 1e-4 1e-4], 0, false);
C(end+1) = mk('near_half_neg',Fh, [r2 -r2 0 0], [r2 -r2 0 0], Z3, Z3, [r2 -r2 0 0], 0, false);

% -- 5) int16-Saettigung F/Omega/tau (jenseits fs=[40,10,2]) --
C(end+1) = mk('sat_F_hi',  50,   qI,qI,Z3,Z3, qI, 0, false);
C(end+1) = mk('sat_F_lo', -50,   qI,qI,Z3,Z3, qI, 0, false);
C(end+1) = mk('sat_Om',    Fh,   qI,qI,[20 -20 15],Z3, qI, 0, false);
C(end+1) = mk('sat_tau',   Fh,   qI,qI,Z3,[5 -5 3], qI, 0, false);
C(end+1) = mk('sat_all',   99,   qI,qI,[99 -99 99],[9 -9 9], qI, 0, false);

% -- 6) estop-Stufen + ack --
C(end+1) = mk('estop1',    Fh, qI,qI,Z3,Z3, qI, 1, false);
C(end+1) = mk('estop2',    Fh, qI,qI,Z3,Z3, qI, 2, true);
C(end+1) = mk('estop2_na', Fh, qI,qI,Z3,Z3, qI, 2, false);

% -- 7) Zufall: unit-Quats + Werte im/leicht ueber Bereich --
rng(12345,'twister');
Nrand = 200;
for i = 1:Nrand
    qd = randn(1,4); qd = qd/norm(qd);
    qr = randn(1,4); qr = qr/norm(qr);
    qe = randn(1,4); qe = qe/norm(qe);
    F  = 45*rand;                       % 0..45 N (bis knapp ueber fs=40)
    Om = 12*(2*rand(1,3)-1);            % +-12 rad/s (bis ueber fs=10)
    tr = 2.4*(2*rand(1,3)-1);           % +-2.4 N*m (bis ueber fs=2)
    C(end+1) = mk(sprintf('rand%03d',i), F, qd,qr,Om,tr,qe, mod(i,3), logical(mod(i,2))); %#ok<SAGROW>
end

%% --- durch link_tx -> link_rx jagen -----------------------------------------
rows = cell(numel(C),1);
for i = 1:numel(C)
    c = C(i);
    cmd_in = struct('F_des',c.F, ...
                    'q_des',c.qd(:), 'q_ref',c.qr(:), 'q_ext',c.qe(:), ...
                    'Omega_ref',c.Om(:), 'tau_ref',c.tr(:), ...
                    'estop',uint8(c.estop), 'ack',logical(c.ack));

    [pkt_i16, pkt_q, flags] = link_tx(cmd_in, link_params);   % pdrop=0 -> = aktuelles Paket
    rx = link_rx(pkt_i16, pkt_q, flags, link_params);

    nums = [ c.F, c.qd, c.qr, c.Om, c.tr, c.qe, double(c.estop), double(c.ack), ...
             double(pkt_i16(:).'), double(pkt_q(:).'), double(flags(:).'), ...
             rx.F_des, rx.q_des(:).', rx.q_ref(:).', rx.Omega_ref(:).', ...
             rx.tau_ref(:).', rx.q_ext(:).', double(rx.estop), double(rx.ack) ];
    rows{i} = [{c.id}, num2cell(nums)];
end

%% --- CSV schreiben -----------------------------------------------------------
outCsv = fullfile(here, '..', 'data', 'link_codec_golden.csv');
if ~isfolder(fileparts(outCsv)); mkdir(fileparts(outCsv)); end

hdr = [ {'id'}, ...
    strc('in_F'), strc('in_qd',4), strc('in_qr',4), strc('in_Om',3), strc('in_tr',3), strc('in_qe',4), strc('in_estop'), strc('in_ack'), ...
    strc('tx_i16',7), strc('tx_q',3), strc('tx_flags',2), ...
    strc('rx_F'), strc('rx_qd',4), strc('rx_qr',4), strc('rx_Om',3), strc('rx_tr',3), strc('rx_qe',4), strc('rx_estop'), strc('rx_ack') ];

fid = fopen(outCsv,'w'); assert(fid>0,'CSV nicht schreibbar: %s',outCsv);
fprintf(fid,'%s', hdr{1}); fprintf(fid,',%s', hdr{2:end}); fprintf(fid,'\n');
for i = 1:numel(rows)
    r = rows{i};
    fprintf(fid,'%s', r{1});
    for k = 2:numel(r); fprintf(fid,',%.17g', r{k}); end
    fprintf(fid,'\n');
end
fclose(fid);
fprintf('link_codec_golden geschrieben: %s  (%d Zeilen, %d Datenspalten)\n', ...
        outCsv, numel(rows), numel(hdr)-1);

%% --- lokale Helfer -----------------------------------------------------------
function s = mk(id,F,qd,qr,Om,tr,qe,estop,ack)
% Feldreihenfolge muss zur C-Praeallokation passen (Zuweisung per Position).
    s = struct('id',id,'F',F, ...
               'qd',qd(:).','qr',qr(:).', ...
               'Om',Om(:).','tr',tr(:).','qe',qe(:).', ...
               'estop',estop,'ack',ack);
end

function c = strc(base, n)
% Spaltennamen: base (n=1) oder base1..baseN.
    if nargin < 2 || n == 1
        c = {base};
    else
        c = arrayfun(@(k) sprintf('%s%d',base,k), 1:n, 'UniformOutput', false);
    end
end
