import struct
from satellite_communicator import SatelliteCommunicator

# Assuming SatelliteCommunicator class code is already defined and imported as you provided earlier.

if __name__ == "__main__":
    # Create an instance of the SatelliteCommunicator
    communicator = SatelliteCommunicator(port="/dev/ttyUSB1", baudrate=115200)

    # Send Arm/Disarm command (Assuming 0x01 means Arm)
    communicator.send_message(communicator.MSG_ID_ARM_DISARM, b"\x01")

    # Send Navigate GPS command with example coordinates (Latitude, Longitude)
    latitude = 34.052235  # Example latitude
    longitude = -118.243683  # Example longitude
    gps_data = struct.pack(">ff", latitude, longitude)
    communicator.send_message(communicator.MSG_ID_NAVIGATE_GPS, gps_data)

    # Notify task completion
    communicator.task_completed()

    # Set a new stage (Example stage 1)
    communicator.send_message(communicator.MSG_ID_SET_STAGE, b"\x01")

    # Locate Aruco Tags command
    communicator.send_message(communicator.MSG_ID_LOCATE_ARUCO_TAGS, b"")

    # Send a detection command (assuming no additional body data is needed)
    communicator.send_message(communicator.MSG_ID_DETECTION, b"")

    # Set parameters command (assuming no additional body data is needed)
    communicator.send_message(communicator.MSG_ID_SET_PARAMETERS, b"")
