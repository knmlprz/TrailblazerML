import serial
import struct


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


def send_message(serial_port, message_id, body):
    start_byte = 0x7E
    body_length = len(body)
    header = struct.pack('BBB', start_byte, message_id, body_length)
    checksum = crc16_ccitt(header + body)
    message = header + body + struct.pack('>H', checksum)
    serial_port.write(message)
    print("Message sent")


def main():
    port = '/dev/ttyUSB0'  # Adjust this as per your connection
    baudrate = 9600
    with serial.Serial(port, baudrate, timeout=1) as ser:
        # Example: Send a "Navigate GPS" command
        message_id = 0x02  # Navigate GPS message ID
        latitude = 34.052235  # Example latitude
        longitude = -118.243683  # Example longitude
        body = struct.pack('>ff', latitude, longitude)

        send_message(ser, message_id, body)

        # You can add more commands here as needed


if __name__ == '__main__':
    main()
