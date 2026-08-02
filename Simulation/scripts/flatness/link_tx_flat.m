function [pkt_i16, pkt_q, flags] = link_tx_flat(cmd_in, link_flat_params)
%#codegen
% link_tx_flat  TX-Seite des Funkkanals (GCS->Drohne), FLATNESS-Variante @ Ts_gcs.
%   Vektoren: [mocap_pos; p_ref; v_ref; a_ref; j_ref; s_ref; yaw_ref] -> int16
%   (21x1), saturiert. q_ext -> smallest-three (uint32). flags: [estop; ack].
%   Bernoulli-Paketverlust (xorshift32): bei Verlust ganzes Paket halten (ZOH).
%
%   Braucht pack_quat_sm3.m auf dem MATLAB-Pfad.

    persistent last_i16 last_q last_flags rs init_done
    if isempty(init_done)
        last_i16   = reshape(int16(link_flat_params.pkt_init(1:21)), 21, 1);
        last_q     = uint32(link_flat_params.q_init(1));
        last_flags = double(link_flat_params.flags_init);
        rs         = uint32(link_flat_params.seed);
        init_done  = true;
    end

    % --- int16-Teil: 21x1 ---
    v = [ reshape(double(cmd_in.mocap_pos), 3, 1); ...
          reshape(double(cmd_in.p_ref),     3, 1); ...
          reshape(double(cmd_in.v_ref),     3, 1); ...
          reshape(double(cmd_in.a_ref),     3, 1); ...
          reshape(double(cmd_in.j_ref),     3, 1); ...
          reshape(double(cmd_in.s_ref),     3, 1); ...
          reshape(double(cmd_in.yaw_ref),   3, 1) ];
    fs   = reshape(double(link_flat_params.fs), 21, 1);
    qmax = double(link_flat_params.qmax);
    qmin = double(link_flat_params.qmin);
    lsb  = fs / qmax;
    qi      = min(max(round(v ./ lsb), qmin), qmax);
    i16_now = reshape(int16(qi), 21, 1);

    % --- q_ext: smallest-three (scalar-first [w x y z]) ---
    q_now = pack_quat_sm3(reshape(double(cmd_in.q_ext), 4, 1));

    % --- flags (verlustfrei quantisierungsfrei) ---
    flags_now = [double(cmd_in.estop); double(cmd_in.ack)];

    % --- Bernoulli-Verlust: ein Zufallswert entscheidet ueber das ganze Paket ---
    [u, rs] = xorshift01(rs);
    if u >= link_flat_params.pdrop
        last_i16   = i16_now;
        last_q     = q_now;
        last_flags = flags_now;
    end

    pkt_i16 = reshape(last_i16, 21, 1);
    pkt_q   = last_q;
    flags   = reshape(last_flags, 2, 1);
end

function [u, s] = xorshift01(s)
% xorshift32 -> u in [0,1). Seed != 0 erforderlich.
    s = bitxor(s, bitshift(s,  13));
    s = bitxor(s, bitshift(s, -17));
    s = bitxor(s, bitshift(s,   5));
    u = double(s) / 4294967296;
end
