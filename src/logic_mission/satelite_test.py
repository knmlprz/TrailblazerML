import serial
import struct

START_BYTE = 0x7E

MSG_ID_ACK = 0x00
MSG_ID_ARM_DISARM = 0x01
MSG_ID_NAVIGATE_GPS = 0x02
MSG_ID_TASK_COMPLETED = 0x03
MSG_ID_SET_STAGE = 0x04
MSG_ID_LOCATE_ARUCO_TAGS = 0x07
MSG_ID_DETECTION = 0x09
MSG_ID_SET_PARAMETERS = 0x0A


def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
    return crc & 0xFFFF

def send_message(ser, message_id, body):
    body_length = len(body)
    header = struct.pack('BBB', START_BYTE, message_id, body_length)
    checksum = crc16_ccitt(header + body)
    message = header + body + struct.pack('>H', checksum)
    ser.serial_port.write(message)


def main():
    port = '/dev/ttyUSB1'  # Adjust this as per your connection
    baudrate = 115200
    with serial.Serial(port, baudrate, timeout=1) as ser:
        send_message(ser, 0x01, b'')
        # Example: Send a "Navigate GPS" command
        message_id = 0x01# Navigate GPS message ID
        latitude = 34.052235  # Example latitude
        longitude = -118.243683  # Example longitude
        body = struct.pack('>ff', latitude, longitude)
        #body =0x01
        body =  b'0x01'
        send_message(ser, message_id, body)

        # You can add more commands here as needed


if __name__ == '__main__':
    main()
