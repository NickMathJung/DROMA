function [pkt_i16, pkt_q, flags] = link_tx(cmd_in, link_params)
%#codegen
% link_tx  TX-Seite des Funkkanals (GCS->Drohne) @ Ts_gcs.
%   Skalare/Vektoren: [F_des; Omega_ref(3); tau_ref(3)] -> int16 (7x1), saturiert.
%   Quaternionen: q_des, q_ref, q_ext -> smallest-three (uint32 3x1).
%   flags: [estop(0/1/2); ack].
%   Bernoulli-Paketverlust (xorshift32): bei Verlust das ganze Paket halten (ZOH);
%   int16-Teil, Quat-Teil und flags gehen gemeinsam raus.
%
%   Braucht pack_quat_sm3.m auf dem MATLAB-Pfad.

    persistent last_i16 last_q last_flags rs init_done
    if isempty(init_done)
        last_i16   = reshape(int16(link_params.pkt_init(1:7)), 7, 1);
        id_code    = pack_quat_sm3([1;0;0;0]);          % Identitaets-Quat als Startwert
        last_q     = [id_code; id_code; id_code];
        last_flags = double(link_params.flags_init);
        rs         = uint32(link_params.seed);
        init_done  = true;
    end

    % --- int16-Teil: [F_des; Omega_ref(3); tau_ref(3)] = 7x1 ---
    F    = double(cmd_in.F_des);
    Om   = reshape(double(cmd_in.Omega_ref), 3, 1);
    tr   = reshape(double(cmd_in.tau_ref),   3, 1);
    v    = reshape([F; Om; tr], 7, 1);
    fs   = reshape(double(link_params.fs), 7, 1);
    qmax = double(link_params.qmax);
    qmin = double(link_params.qmin);
    lsb  = fs / qmax;
    qi       = min(max(round(v ./ lsb), qmin), qmax);
    i16_now  = reshape(int16(qi), 7, 1);

    % --- Quaternionen: smallest-three (scalar-first [w x y z]) ---
    q_now    = zeros(3,1,'uint32');
    q_now(1) = pack_quat_sm3(reshape(double(cmd_in.q_des), 4, 1));
    q_now(2) = pack_quat_sm3(reshape(double(cmd_in.q_ref), 4, 1));
    q_now(3) = pack_quat_sm3(reshape(double(cmd_in.q_ext), 4, 1));

    % --- flags (verlustfrei) ---
    flags_now = [double(cmd_in.estop); double(cmd_in.ack)];

    % --- Bernoulli-Verlust: ein Zufallswert entscheidet ueber das ganze Paket ---
    [u, rs] = xorshift01(rs);
    if u >= link_params.pdrop
        last_i16   = i16_now;
        last_q     = q_now;
        last_flags = flags_now;
    end

    pkt_i16 = reshape(last_i16, 7, 1);
    pkt_q   = reshape(last_q,   3, 1);
    flags   = reshape(last_flags, 2, 1);
end

function [u, s] = xorshift01(s)
% xorshift32 -> u in [0,1). Seed != 0 erforderlich.
    s = bitxor(s, bitshift(s,  13));
    s = bitxor(s, bitshift(s, -17));
    s = bitxor(s, bitshift(s,   5));
    u = double(s) / 4294967296;
end