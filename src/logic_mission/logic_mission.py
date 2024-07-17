from satellite_communicator import SatelliteCommunicator

class Mission:
    def __init__(self):
        self.satellite_communicator = SatelliteCommunicator(port="/dev/ttyAMA0", baudrate=9600)
        self.autonomy_is_on = False
    def loop_mission(self):
        while self.autonomy_is_on:
            self.satellite_communicator.read_message()
            self

