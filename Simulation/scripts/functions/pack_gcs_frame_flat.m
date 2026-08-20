function frame = pack_gcs_frame_flat(cmd, id)
%pack_gcs_frame_flat  GS-USB-Frame der FLATNESS-Variante (106 B).
%   [sync|id|Bus_Cmd_flat(float32)|estop|ack|crc8].
%
%   cmd: struct mit mocap_pos(3), q_ext(4), p_ref(3), v_ref(3), a_ref(3),
%        j_ref(3), s_ref(3), yaw_ref(3), estop(0/1/2), ack(0/1).
%   id: Ziel-Drohne (uint8, 0..15).
%   frame: 1x106 uint8, little-endian
%#codegen
    vals = single([ reshape(double(cmd.mocap_pos),3,1); ...
                    reshape(double(cmd.q_ext),4,1); ...
                    reshape(double(cmd.p_ref),3,1); ...
                    reshape(double(cmd.v_ref),3,1); ...
                    reshape(double(cmd.a_ref),3,1); ...
                    reshape(double(cmd.j_ref),3,1); ...
                    reshape(double(cmd.s_ref),3,1); ...
                    reshape(double(cmd.yaw_ref),3,1) ]).'; % 1x25 single

    frame = zeros(1,106,'uint8');
    frame(1) = uint8(170); % 0xAA
    frame(2) = uint8(85);  % 0x55
    frame(3) = uint8(id);
    frame(4:103) = typecast(vals, 'uint8'); % 25x float32 LE = 100 Bytes
    frame(104) = uint8(cmd.estop);
    frame(105) = uint8(cmd.ack ~= 0);
    frame(106) = crc8(frame(3:105)); % ueber id + Payload + estop + ack
end

function c = crc8(bytes)
% CRC-8/SMBus: Poly 0x07, Init 0x00.
    c = uint8(0);
    for k = 1:numel(bytes)
        c = bitxor(c, uint8(bytes(k)));
        for b = 1:8
            if bitand(c, uint8(128))
                c = bitxor(uint8(bitshift(c,1)), uint8(7));
            else
                c = uint8(bitshift(c,1));
            end
        end
    end
end
