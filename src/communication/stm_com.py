import pyserial
import numpy as np

class STMCom:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=1)
        self.autonomy = False

    def update(self):
        pass

    def send_command(ser, left, right):

        uint8_t_left = np.uint8(left)
        uint8_t_right = np.uint8(right)
        checksum = (uint8_t_left + uint8_t_right) % 256
        command = f"*{uint8_t_left}{uint8_t_right}{checksum}".encode()
        ser.write(command)

    def read_response(ser):
        response = ser.read(3)
        if len(response) == 3:
            left_speed = response[0]
            right_speed = response[1]
            received_checksum = response[2]

            calculated_checksum = (left_speed + right_speed) % 256

            if calculated_checksum == received_checksum:
                print(f"Left speed: {left_speed}, Right speed: {right_speed}")
            else:
                print("Błąd: Nieprawidłowa suma kontrolna.")
    def receive(self, size):
        return self.ser.read(size)

    def close(self):
        self.ser.close()

