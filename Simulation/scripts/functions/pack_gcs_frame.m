function frame = pack_gcs_frame(cmd, id)
%pack_gcs_frame  GS-USB-Frame [sync|id|Bus_Cmd(float32)|estop|ack|crc8] (82 B).
%   Spiegelt gcs_frame.hpp (build) und ist zugleich die Byte-Spec fuer die
%   Simulink-Serial-Send-Seite: die GCS muss genau dieses Layout erzeugen.
%
%   cmd: struct mit F_des, q_des(4), q_ref(4), Omega_ref(3), tau_ref(3),
%        q_ext(4), estop(0/1/2), ack(0/1). id: Ziel-Drohne (uint8, 0..15).
%   frame: 1x82 uint8, little-endian
%#codegen
    vals = single([ double(cmd.F_des); ...
                    reshape(double(cmd.q_des),4,1); ...
                    reshape(double(cmd.q_ref),4,1); ...
                    reshape(double(cmd.Omega_ref),3,1); ...
                    reshape(double(cmd.tau_ref),3,1); ...
                    reshape(double(cmd.q_ext),4,1) ]).';   % 1x19 single

    frame = zeros(1,82,'uint8');
    frame(1) = uint8(170); % 0xAA
    frame(2) = uint8(85); % 0x55
    frame(3) = uint8(id);
    frame(4:79) = typecast(vals, 'uint8'); % 19x float32 LE = 76 Bytes
    frame(80) = uint8(cmd.estop);
    frame(81) = uint8(cmd.ack ~= 0);
    frame(82) = crc8(frame(3:81)); % ueber id + Payload + estop + ack
end

function c = crc8(bytes)
% CRC-8/SMBus: Poly 0x07, Init 0x00 (bitgleich zu gcs::detail::crc8).
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
