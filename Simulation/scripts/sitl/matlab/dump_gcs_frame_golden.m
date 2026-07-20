%% dump_gcs_frame_golden.m — Golden fuer den GS-Frame-Cross-Check.
%  Pro Zeile ein Bus_Cmd samt Ziel-id, durch pack_gcs_frame zum 82-B-Frame.
%  Der Host-Test test_gcs_frame parst die Bytes via gcs::parse (gcs_frame.hpp)
%  und vergleicht gegen die float32-gerundeten Bus_Cmd-Werte; das zeigt, dass
%  Simulink-Schreiber und Sende-Teensy-Leser identisch sind.
%
%  Spalten (nach id-Label): in_id, in_F, in_qd1..4, in_qr1..4, in_Om1..3,
%    in_tr1..3, in_qe1..4, in_estop, in_ack, frame0..frame81.

here        = fileparts(mfilename('fullpath'));
scriptsRoot = fullfile(here, '..', '..');
addpath(scriptsRoot, fullfile(scriptsRoot,'functions'), fullfile(scriptsRoot,'init'));

qI = [1 0 0 0]; Z3 = [0 0 0];
C = struct('id',{},'tid',{},'F',{},'qd',{},'qr',{},'Om',{},'tr',{},'qe',{},'estop',{},'ack',{});

C(end+1) = mk('hover',      0,  9.4666, qI,qI,Z3,Z3, qI, 0, false);
C(end+1) = mk('zeroF',      7,  0,      qI,qI,Z3,Z3, qI, 0, true);
C(end+1) = mk('estop1',     3,  9.4666, qI,qI,Z3,Z3, qI, 1, false);
C(end+1) = mk('estop2_ack', 15, 9.4666, qI,qI,Z3,Z3, qI, 2, true);
C(end+1) = mk('bigvals',    1,  40, [0.5 0.5 0.5 0.5],[0 1 0 0],[9 -9 9],[1.9 -1.9 1], [0 0 1 0], 2, false);
C(end+1) = mk('neg',        8, -12.5, [-1 0 0 0],qI,[-20 20 -15],[-5 5 -3], qI, 0, true);

rng(2024,'twister');
for i = 1:60
    qd = randn(1,4); qd=qd/norm(qd);
    qr = randn(1,4); qr=qr/norm(qr);
    qe = randn(1,4); qe=qe/norm(qe);
    F  = 50*rand; Om = 25*(2*rand(1,3)-1); tr = 5*(2*rand(1,3)-1);
    C(end+1) = mk(sprintf('rand%03d',i), mod(i,16), F, qd,qr,Om,tr,qe, mod(i,3), logical(mod(i,2))); %#ok<SAGROW>
end

rows = cell(numel(C),1);
for i = 1:numel(C)
    c = C(i);
    cmd = struct('F_des',c.F,'q_des',c.qd(:),'q_ref',c.qr(:),'q_ext',c.qe(:), ...
                 'Omega_ref',c.Om(:),'tau_ref',c.tr(:),'estop',uint8(c.estop),'ack',logical(c.ack));
    frame = pack_gcs_frame(cmd, c.tid);
    nums = [ double(c.tid), c.F, c.qd, c.qr, c.Om, c.tr, c.qe, double(c.estop), double(c.ack), double(frame) ];
    rows{i} = [{c.id}, num2cell(nums)];
end

outCsv = fullfile(here,'..','data','gcs_frame_golden.csv');
if ~isfolder(fileparts(outCsv)); mkdir(fileparts(outCsv)); end
hdr = [ {'id'}, strc('in_id'), strc('in_F'), strc('in_qd',4), strc('in_qr',4), ...
        strc('in_Om',3), strc('in_tr',3), strc('in_qe',4), strc('in_estop'), strc('in_ack'), ...
        strc('frame',82) ];
fid = fopen(outCsv,'w'); assert(fid>0,'CSV nicht schreibbar: %s',outCsv);
fprintf(fid,'%s',hdr{1}); fprintf(fid,',%s',hdr{2:end}); fprintf(fid,'\n');
for i = 1:numel(rows)
    r = rows{i};
    fprintf(fid,'%s',r{1});
    for k = 2:numel(r); fprintf(fid,',%.17g',r{k}); end
    fprintf(fid,'\n');
end
fclose(fid);
fprintf('gcs_frame_golden geschrieben: %s  (%d Zeilen, %d Datenspalten)\n', outCsv, numel(rows), numel(hdr)-1);

function s = mk(id,tid,F,qd,qr,Om,tr,qe,estop,ack)
    s = struct('id',id,'tid',tid,'F',F,'qd',qd(:).','qr',qr(:).', ...
               'Om',Om(:).','tr',tr(:).','qe',qe(:).','estop',estop,'ack',ack);
end
function c = strc(base,n)
    if nargin<2 || n==1, c={base}; else, c=arrayfun(@(k)sprintf('%s%d',base,k),1:n,'UniformOutput',false); end
end
