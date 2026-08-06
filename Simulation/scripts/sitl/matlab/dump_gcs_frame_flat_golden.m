%% dump_gcs_frame_flat_golden.m — Golden fuer den GS-Frame-Cross-Check (FLATNESS).
%  Pro Zeile ein Bus_Cmd_flat samt Ziel-id, durch pack_gcs_frame_flat zum 106-B-
%  Frame. Der Host-Test test_gcs_frame_flat parst die Bytes via gcsf::parse
%  (gcs_frame_flat.hpp) und vergleicht gegen die float32-gerundeten Werte; das
%  zeigt, dass Simulink-Schreiber und Sende-Teensy-Leser identisch sind.
%
%  Spalten (nach id-Label): in_id, in_moc1..3, in_qe1..4, in_p1..3, in_v1..3,
%    in_a1..3, in_j1..3, in_s1..3, in_yaw1..3, in_estop, in_ack, frame0..frame105.

here        = fileparts(mfilename('fullpath'));
scriptsRoot = fullfile(here, '..', '..');
addpath(scriptsRoot, fullfile(scriptsRoot,'functions'), fullfile(scriptsRoot,'init'));

qI = [1 0 0 0]; Z3 = [0 0 0];
C = struct('id',{},'tid',{},'moc',{},'qe',{},'p',{},'v',{},'a',{},'j',{},'s',{},'yaw',{},'estop',{},'ack',{});

C(end+1) = mkf('hover',      0, [0 0 1], qI, [0 0 1], Z3,Z3,Z3,Z3, Z3, 0, false);
C(end+1) = mkf('zeros',      7, Z3, qI, Z3, Z3,Z3,Z3,Z3, Z3, 0, true);
C(end+1) = mkf('estop1',     3, [0 0 1], qI, [0 0 1], Z3,Z3,Z3,Z3, Z3, 1, false);
C(end+1) = mkf('estop2_ack',15, [0 0 1], qI, [0 0 1], Z3,Z3,Z3,Z3, Z3, 2, true);
C(end+1) = mkf('mocap_inv',  2, [0 0 1], [0 0 0 0], [0 0 1], Z3,Z3,Z3,Z3, Z3, 0, false);
C(end+1) = mkf('bigvals',    1, [5 -5 3], [0.5 0.5 0.5 0.5], [5 -5 3], [9 -9 9], ...
                                [40 -40 30], [150 -150 100], [1500 -1500 900], [3 -15 150], 2, false);
C(end+1) = mkf('neg',        8, [-2 -2 -2], [-1 0 0 0], [-2 -2 -2], [-9 9 -9], ...
                                [-30 30 -20], [-100 100 -80], [-900 900 -700], [-3 15 -150], 0, true);

rng(2026,'twister');
for i = 1:60
    qe  = randn(1,4); qe = qe/norm(qe);
    moc = 25*(2*rand(1,3)-1);  p = 25*(2*rand(1,3)-1);  v = 25*(2*rand(1,3)-1);
    a   = 60*(2*rand(1,3)-1);  j = 250*(2*rand(1,3)-1); s = 2500*(2*rand(1,3)-1);
    yaw = [5*(2*rand-1), 25*(2*rand-1), 250*(2*rand-1)];
    C(end+1) = mkf(sprintf('rand%03d',i), mod(i,16), moc, qe, p, v, a, j, s, yaw, ...
                   mod(i,3), logical(mod(i,2))); %#ok<SAGROW>
end

rows = cell(numel(C),1);
for i = 1:numel(C)
    c = C(i);
    cmd = struct('mocap_pos',c.moc(:),'q_ext',c.qe(:),'p_ref',c.p(:),'v_ref',c.v(:), ...
                 'a_ref',c.a(:),'j_ref',c.j(:),'s_ref',c.s(:),'yaw_ref',c.yaw(:), ...
                 'estop',uint8(c.estop),'ack',logical(c.ack));
    frame = pack_gcs_frame_flat(cmd, c.tid);
    nums = [ double(c.tid), c.moc, c.qe, c.p, c.v, c.a, c.j, c.s, c.yaw, ...
             double(c.estop), double(c.ack), double(frame) ];
    rows{i} = [{c.id}, num2cell(nums)];
end

outCsv = fullfile(here,'..','data','gcs_frame_flat_golden.csv');
if ~isfolder(fileparts(outCsv)); mkdir(fileparts(outCsv)); end
hdr = [ {'id'}, strc('in_id'), strc('in_moc',3), strc('in_qe',4), strc('in_p',3), ...
        strc('in_v',3), strc('in_a',3), strc('in_j',3), strc('in_s',3), strc('in_yaw',3), ...
        strc('in_estop'), strc('in_ack'), strc('frame',106) ];
fid = fopen(outCsv,'w'); assert(fid>0,'CSV nicht schreibbar: %s',outCsv);
fprintf(fid,'%s',hdr{1}); fprintf(fid,',%s',hdr{2:end}); fprintf(fid,'\n');
for i = 1:numel(rows)
    r = rows{i};
    fprintf(fid,'%s',r{1});
    for k = 2:numel(r); fprintf(fid,',%.17g',r{k}); end
    fprintf(fid,'\n');
end
fclose(fid);
fprintf('gcs_frame_flat_golden geschrieben: %s  (%d Zeilen, %d Datenspalten)\n', outCsv, numel(rows), numel(hdr)-1);

function s = mkf(id,tid,moc,qe,p,v,a,j,sn,yaw,estop,ack)
    s = struct('id',id,'tid',tid,'moc',moc(:).','qe',qe(:).','p',p(:).','v',v(:).', ...
               'a',a(:).','j',j(:).','s',sn(:).','yaw',yaw(:).','estop',estop,'ack',ack);
end
function c = strc(base,n)
    if nargin<2 || n==1, c={base}; else, c=arrayfun(@(k)sprintf('%s%d',base,k),1:n,'UniformOutput',false); end
end
