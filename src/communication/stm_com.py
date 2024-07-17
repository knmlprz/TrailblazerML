import serial
class STMCom:
    def __init__(self, port, serial_port=None):
        if serial_port:
            self.ser = serial_port
        else:
            self.ser = serial.Serial(port, 115200, timeout=1)
        self.autonomy = False
        self.left_speed = 79
        self.right_speed = 79
        self.stop_rover = False

    def update(self, left_jetson, right_jeston):
        self.left_speed = self.scale_value(left_jetson, (-1, 1), (32, 126))
        self.right_speed = self.scale_value(right_jeston, (-1, 1), (32, 126))
        self.read_response()
        return self.autonomy
    def map_speed(self, left_speed, right_speed):
        #pabing from -1 to 1 to 0 to 1 t0 32 to 126
        left_speed = int(map(left_speed, -1, 1, 32, 126))
    def send_command(self):
        if self.stop_rover:
            self.left_speed = 79
            self.right_speed = 79
        start_byte = ord('*')
        command = bytes([start_byte, self.left_speed, self.right_speed])
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

    def inject_response(self, data):
        return self.read_response()

#
# from unittest.mock import MagicMock
# import serial
#
# # Przykładowa funkcja testująca
# import unittest
# from unittest.mock import MagicMock
# from stm_com import STMCom
#
#
# class TestSTMCom(unittest.TestCase):
#     def test_read_response_with_valid_data(self):
#         # Mock serial port
#         mock_serial = MagicMock()
#
#         # Set up the mock to simulate incoming data that the read_response method expects
#         # Simulating data: '*' as start byte, 'Y' as data byte, and valid checksum
#         start_byte = ord('*')
#         data_byte = ord('Y')
#         checksum = (start_byte + data_byte) & 0xFF
#         mock_serial.read.side_effect = [
#             bytes([start_byte]),  # Response for read(1)
#             bytes([data_byte, checksum])  # Response for read(2)
#         ]
#
#         # Create an instance of STMCom using the mocked serial port
#         stm = STMCom(port="dummy_port", serial_port=mock_serial)
#
#         # Call read_response and assert its effects
#         stm.read_response()
#         self.assertTrue(stm.autonomy)
#         print(f"Autonomy status: {stm.autonomy}")
#
#     def test_read_response_with_invalid_checksum(self):
#         # Mock serial port
#         mock_serial = MagicMock()
#
#         # Simulating data: '*' as start byte, 'Y' as data byte, but incorrect checksum
#         start_byte = ord('*')
#         data_byte = ord('Y')
#         incorrect_checksum = (start_byte + data_byte) & 0xFF  # Deliberately incorrect
#         mock_serial.read.side_effect = [
#             bytes([start_byte]),  # Response for read(1)
#             bytes([data_byte, incorrect_checksum])  # Response for read(2)
#         ]
#
#         # Create an instance of STMCom using the mocked serial port
#         stm = STMCom(port="dummy_port", serial_port=mock_serial)
#
#         # Call read_response and assert its effects
#         stm.read_response()
#         self.assertFalse(stm.autonomy)
#         print(f"Autonomy status: {stm.autonomy}")
#
#
#     def tearDown(self):
#         # This method will be called after each test
#         print("Test completed.")
#
#
# # Run the tests
# if __name__ == '__main__':
#     unittest.main()
