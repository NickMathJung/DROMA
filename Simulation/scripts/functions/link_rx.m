function cmd_out = link_rx(pkt, flags, link_params)
%#codegen
% RX-Seite Funkkanal (Drohne) @ Ts_inner.
%   int16-Paket (19x1) -> dequantisieren -> Bus_Cmd
%   Quaternionen werden re-normiert (Quant-Rauschen entfernen).
p    = reshape(double(pkt), 19, 1);
fs   = reshape(double(link_params.fs), 19, 1);
qmax = double(link_params.qmax);
lsb  = fs / qmax;
v    = reshape(p .* lsb, 19, 1);
F    = v(1);
qd   = reshape(v(2:5),   4, 1);
qr   = reshape(v(6:9),   4, 1);
Om   = reshape(v(10:12), 3, 1);
tr   = reshape(v(13:15), 3, 1);
qe   = reshape(v(16:19), 4, 1);
n_qd = sqrt(qd.'*qd); 
if n_qd < 1e-12
    n_qd = 1;
end
n_qr = sqrt(qr.'*qr); 
if n_qr < 1e-12 
    n_qr = 1; 
end
n_qe = sqrt(qe.'*qe); 
if n_qe < 1e-12 
    n_qe = 1; 
end

cmd_out.F_des = F;
cmd_out.q_des = qd / n_qd;
cmd_out.q_ref = qr / n_qr;
cmd_out.Omega_ref = Om;
cmd_out.tau_ref = tr;
cmd_out.q_ext = qe / n_qe;
cmd_out.estop = uint8(flags(1));
cmd_out.ack = flags(2) > 0.5;
end
