from satellite_communicator import SatelliteCommunicator

class Mission:
    def __init__(self):
        self.satellite_communicator = SatelliteCommunicator()

    def run(self):
        self.satellite_communicator.read_message()

