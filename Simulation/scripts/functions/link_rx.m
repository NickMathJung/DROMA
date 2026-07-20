function cmd_out = link_rx(pkt_i16, pkt_q, flags, link_params)
%#codegen
% link_rx  RX-Seite des Funkkanals (Drohne) @ Ts_inner.
%   int16 (7x1) -> dequantisieren -> F_des, Omega_ref, tau_ref
%   uint32 (3x1) -> smallest-three -> q_des, q_ref, q_ext (in unpack re-normiert)
%
%   Braucht unpack_quat_sm3.m auf dem MATLAB-Pfad.
%   Feldreihenfolge von cmd_out == Bus_Cmd (setup_buses.m):
%     F_des, q_des, q_ref, Omega_ref, tau_ref, q_ext, estop, ack.

    p    = reshape(double(pkt_i16), 7, 1);
    fs   = reshape(double(link_params.fs), 7, 1);
    qmax = double(link_params.qmax);
    lsb  = fs / qmax;
    v    = reshape(p .* lsb, 7, 1);

    cmd_out.F_des     = v(1);
    cmd_out.q_des     = unpack_quat_sm3(pkt_q(1));
    cmd_out.q_ref     = unpack_quat_sm3(pkt_q(2));
    cmd_out.Omega_ref = reshape(v(2:4), 3, 1);
    cmd_out.tau_ref   = reshape(v(5:7), 3, 1);
    cmd_out.q_ext     = unpack_quat_sm3(pkt_q(3));
    cmd_out.estop     = uint8(flags(1));
    cmd_out.ack       = flags(2) > 0.5;
end