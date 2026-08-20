function cmd_out = link_rx_flat(pkt_i16, pkt_q, flags, link_flat_params)
%#codegen
% link_rx_flat  RX-Seite des Funkkanals (Drohne), FLATNESS-Variante @ Ts_inner.
%   int16 (21x1) -> dequantisieren; uint32 -> smallest-three -> q_ext.
%   Feldreihenfolge von cmd_out == Bus_Cmd_flat:
%     mocap_pos, q_ext, p_ref, v_ref, a_ref, j_ref, s_ref, yaw_ref, estop, ack.
%
%   Braucht unpack_quat_sm3.m auf dem MATLAB-Pfad.

    p    = reshape(double(pkt_i16), 21, 1);
    fs   = reshape(double(link_flat_params.fs), 21, 1);
    qmax = double(link_flat_params.qmax);
    lsb  = fs / qmax;
    v    = reshape(p .* lsb, 21, 1);

    cmd_out.mocap_pos = reshape(v(1:3),   3, 1);
    cmd_out.q_ext     = unpack_quat_sm3(pkt_q(1));
    cmd_out.p_ref     = reshape(v(4:6),   3, 1);
    cmd_out.v_ref     = reshape(v(7:9),   3, 1);
    cmd_out.a_ref     = reshape(v(10:12), 3, 1);
    cmd_out.j_ref     = reshape(v(13:15), 3, 1);
    cmd_out.s_ref     = reshape(v(16:18), 3, 1);
    cmd_out.yaw_ref   = reshape(v(19:21), 3, 1);
    cmd_out.estop     = uint8(flags(1));
    cmd_out.ack       = flags(2) > 0.5;
end
