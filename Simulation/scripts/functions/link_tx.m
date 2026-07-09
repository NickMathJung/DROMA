function [pkt, flags] = link_tx(cmd_in, link_params)
%#codegen
% TX-Seite Funkkanal (GCS->Drohne) @ Ts_gcs.
%   int16-Quantisierung mit Saturierung
%   Bernoulli-Paketverlust (xorshift32); bei Verlust letztes Paket halten (ZOH).
    persistent last_pkt last_flags rs init_done
    if isempty(init_done)
        last_pkt = reshape(int16(link_params.pkt_init(1:19)), 19, 1);
        last_flags = double(link_params.flags_init);
        rs = uint32(link_params.seed);
        init_done = true;
    end
    F   = double(cmd_in.F_des);
    qd  = reshape(double(cmd_in.q_des), 4, 1);
    qr  = reshape(double(cmd_in.q_ref), 4, 1);
    Om  = reshape(double(cmd_in.Omega_ref), 3, 1);
    tr  = reshape(double(cmd_in.tau_ref), 3, 1);
    qe  = reshape(double(cmd_in.q_ext), 4, 1);
    v   = reshape([F; qd; qr; Om; tr; qe], 19, 1);
    fs   = reshape(double(link_params.fs), 19, 1);
    qmax = double(link_params.qmax);
    qmin = double(link_params.qmin);
    lsb  = fs / qmax;
    qi      = min(max(round(v ./ lsb), qmin), qmax);
    pkt_now = reshape(int16(qi), 19, 1);
    flags_now = [double(cmd_in.estop); double(cmd_in.ack)];   % [estop; ack]
    [u, rs] = xorshift01(rs);
    if u >= link_params.pdrop
        last_pkt = pkt_now;
        last_flags = flags_now;
    end
    pkt = reshape(last_pkt, 19, 1);
    flags = reshape(last_flags, 2, 1);
end

function [u, s] = xorshift01(s)
% xorshift32 -> u in [0,1). Seed != 0 erforderlich.
s = bitxor(s, bitshift(s,  13));
s = bitxor(s, bitshift(s, -17));
s = bitxor(s, bitshift(s,   5));
u = double(s) / 4294967296;
end
