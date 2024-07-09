import serial
class STMCom:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=1)
        self.autonomy = False
        self.left_speed = 79
        self.right_speed = 79

    def update(self, left_jetson, right_jeston):
        self.left_speed = self.scale_value(left_jetson, (-1, 1), (32, 126))
        self.right_speed = self.scale_value(right_jeston, (-1, 1), (32, 126))
        self.read_response()
        return self.autonomy
    def map_speed(self, left_speed, right_speed):
        #pabing from -1 to 1 to 0 to 1 t0 32 to 126
        left_speed = int(map(left_speed, -1, 1, 32, 126))
    def send_command(self):
        command = bytes(['*', self.left_speed, self.right_speed])
        checksum = sum(command) & 0xFF
        command += bytes([checksum])
        self.ser.write(command)

    def read_response(self):
        start = self.ser.read(1)
        if start == b'*':
            read = self.ser.read(2)
            calculated_checksum = (ord('*') + read[0]) & 0xFF
            if read[0] == ord('Y') and read[1] == calculated_checksum:
                self.autonomy = True
                self.send_command()
                print(f"Autonomy enabled send: left {self.left_speed} right {self.right_speed}")

    def scale_value(self, x, src_range, dst_range):
        scaled = dst_range[0] + ((x - src_range[0]) * (dst_range[1] - dst_range[0]) / (src_range[1] - src_range[0]))
        return int(scaled)
    def close(self):
        self.ser.close()



# Przykład użycia
input_value = -0.90  # Przykładowa wartość do przeskalowania
mapped_value = scale_value(input_value, (-1, 1), (32, 126))
print(f"Przeskalowana wartość: {mapped_value}")