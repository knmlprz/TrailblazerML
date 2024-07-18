import serial
import struct


class STMCom:
    def __init__(self, port, serial_port=None):
        if serial_port:
            self.ser = serial_port
        else:
            self.ser = serial.Serial(port, 115200, timeout=0.5)
        self.autonomy = False
        self.left_speed = 79
        self.right_speed = 79
        self.stop_rover = False
        self.led_r = False
        self.led_g = False
        self.led_y = False
        self.gripper_open = False
        self.byte_light_maini = 0b0000

    def update(self, left_jetson, right_jeston):
        self.left_speed = self.scale_value(left_jetson, (-1, 1), (64, 93))
        self.right_speed = self.scale_value(right_jeston, (-1, 1), (64, 93))
        self.send_command()
        return self.autonomy

    # def send_command(self):
    #     if self.stop_rover:
    #         self.left_speed = 79
    #         self.right_speed = 79
    #     start_byte = ord('*')
    #     command = bytes([start_byte, self.left_speed, self.right_speed])
    #     checksum = sum(command) & 0xFF
    #     command += bytes([checksum])
    #     self.ser.write(command)

    def send_command(self):
        if self.stop_rover:
            self.left_speed = 79
            self.right_speed = 79
        self.byte_light_maini = (
                (1 if self.led_r else 0) |
                (1 if self.led_g else 0) << 1 |
                (1 if self.led_y else 0) << 2 |
                (1 if self.gripper_open else 0) << 3
        )
        command = struct.pack('BBBB', 38, self.right_speed, self.left_speed, self.byte_light_maini)
        checksum = sum(command) & 0xFF
        command += struct.pack('B', checksum)
        # command = f"&{chr(self.right_speed)}{chr(self.left_speed)}{chr(self.byte_light_maini)}"
        # checksum = sum(command.encode()) & 0xFF
        # command += chr(checksum)
        # command = bytearray([self.right_speed, self.left_speed, self.byte_light_maini])
        #
        # checksum = sum(command) & 0xFF
        #
        # command.append(checksum)
        ty = True
        while ty:
            self.ser.write(command)
            print(f"send to stm {command[0]}")
            print(f"send to stm {command[1]}")
            print(f"send to stm {command[2]}")
            print(f"send to stm {command[3]}")
            print(f"send to stm {command[4]}")
            ty = self.read_response()
        print("end")

    def read_response(self):
        print("XX")
        if self.ser.in_waiting > 0:
            start = self.ser.read(1)
            print(f"l1 start {start}")
            if start == b'&':

                read = self.ser.read(2)
                calculated_checksum = (ord('&') + read[0]) & 0xFF
                print(f"l2 start {read}")
                if read[0] == ord('Y') and read[1] == calculated_checksum:
                    print(f"l3 start {read}")
                    return False
        return True

    def scale_value(self, x, src_range, dst_range):
        scaled = dst_range[0] + ((x - src_range[0]) * (dst_range[1] - dst_range[0]) / (src_range[1] - src_range[0]))
        return int(scaled)

    def close(self):
        self.ser.close()
