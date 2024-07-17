import struct
import serial
import time

class SatelliteCommunicator:
    START_BYTE = 0x7E

    MSG_ID_ACK = 0x00
    MSG_ID_ARM_DISARM = 0x01
    MSG_ID_NAVIGATE_GPS = 0x02
    MSG_ID_TASK_COMPLETED = 0x03
    MSG_ID_SET_STAGE = 0x04
    MSG_ID_LOCATE_ARUCO_TAGS = 0x07
    MSG_ID_DETECTION = 0x09
    MSG_ID_SET_PARAMETERS = 0x0A

    def __init__(self, port="/dev/ttyUSB0", baudrate=9600):
        self.serial_port = serial.Serial(port, baudrate, timeout=0.5)
        self.current_stage = 0
        self.log_file = open("communication_log.txt", "a")  # Open a file in append mode

    def write_log(self, message):
        self.log_file.write(message + "\n")  # Write message to file with a newline
        self.log_file.flush()  # Ensure it gets written to disk

    def read_message(self):
        while True:
            print("l0")
            if self.serial_port.in_waiting > 0:
                start_byte = self.serial_port.read(1)
                print(f"l1 start_byte {start_byte.hex()}")
                if start_byte == struct.pack('B', self.START_BYTE):

                    message_id = struct.unpack('B', self.serial_port.read(1))[0]
                    body_length = struct.unpack('B', self.serial_port.read(1))[0]
                    body = self.serial_port.read(body_length)
                    checksum = struct.unpack('>H', self.serial_port.read(2))[0]
                    print(f"l2 message_id {message_id.hex()} body_length {body_length.hex()} body {body.hex()} checksum {checksum}")
                    if self.verify_checksum(
                            start_byte + struct.pack('B', message_id) + struct.pack('B', body_length) + body, checksum):

                        self.process_message(message_id, body)
                        self.send_acknowledge()
                        self.write_log("Message processed: ID {} Body Length {}".format(message_id, body_length))
                        print(f"l3 Message processed: ID {message_id} Body Length {body_length}")
    def send_message(self, message_id, body):
        body_length = len(body)
        header = struct.pack('BBB', self.START_BYTE, message_id, body_length)
        checksum = self.crc16_ccitt(header + body)
        message = header + body + struct.pack('>H', checksum)
        self.serial_port.write(message)
        self.write_log("Sent message: ID {} Body {}".format(message_id, body))

    def send_acknowledge(self):
        self.send_message(self.MSG_ID_ACK, b'')

    def verify_checksum(self, data, received_checksum):
        calculated_checksum = self.crc16_ccitt(data)
        return calculated_checksum == received_checksum

    def crc16_ccitt(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
        return crc & 0xFFFF

    def process_message(self, message_id, body):
        handler_methods = {
            self.MSG_ID_ACK: self.handle_ack,
            self.MSG_ID_ARM_DISARM: self.handle_arm_disarm,
            self.MSG_ID_NAVIGATE_GPS: self.handle_navigate_gps,
            self.MSG_ID_TASK_COMPLETED: self.handle_task_completed,
            self.MSG_ID_SET_STAGE: self.handle_set_stage,
            self.MSG_ID_LOCATE_ARUCO_TAGS: self.handle_locate_aruco_tags,
            self.MSG_ID_DETECTION: self.handle_detection,
            self.MSG_ID_SET_PARAMETERS: self.handle_set_parameters
        }
        handler = handler_methods.get(message_id, lambda x: None)
        handler(body)

    def handle_ack(self, body):
        print("Acknowledge received.")

    def handle_arm_disarm(self, body):
        self.arm_status = struct.unpack('B', body)[0]
        print("Arm/Disarm command processed.")

    def handle_navigate_gps(self, body):
        print("Navigate GPS command processed.")

    def handle_task_completed(self, body):
        print("Task Completed command processed.")

    def task_completed(self):
        self.send_message(self.MSG_ID_TASK_COMPLETED, b'')  # assuming no additional data needed
        print("Sent 'Task Completed' message to host.")

    def handle_set_stage(self, body):
        self.current_stage = struct.unpack('B', body)[0]
        print("Set Stage command processed.")

    def handle_locate_aruco_tags(self, body):
        print("Locate Aruco Tags command processed.")

    def handle_detection(self, body):
        print("Detection command processed.")

    def handle_set_parameters(self, body):
        print("Set Parameters command processed.")


if __name__ == "__main__":
    satellite_communicator = SatelliteCommunicator(port="/dev/ttyUSB0", baudrate=115200)
    while True:
        satellite_communicator.read_message()
    # satellite_communicator.send_acknowledge()
    # satellite_communicator.send_message(SatelliteCommunicator.MSG_ID_ARM_DISARM, b'\x01')
    # satellite_communicator.send_message(SatelliteCommunicator.MSG_ID_NAVIGATE_GPS, struct.pack('>ff', 34.052235, -118.243683))
    # satellite_communicator.task_completed()
    # satellite_communicator.send_message(SatelliteCommunicator.MSG_ID_SET_STAGE, b'\x01')
    # satellite_communicator.send_message(SatelliteCommunicator.MSG_ID_LOCATE_ARUCO_TAGS, b'')
    # satellite_communicator.send_message(SatelliteCommunicator.MSG_ID_DETECTION, b'')
    # satellite_communicator.send_message(SatelliteCommunicator.MSG_ID_SET_PARAMETERS, b'')
