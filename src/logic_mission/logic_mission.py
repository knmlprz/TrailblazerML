from satellite_communicator import SatelliteCommunicator
from ..communication.stm_com import STMCom
class Mission:
    def __init__(self):
        self.stm_com = STMCom(port="/dev/ttyACM0")
        self.satellite_communicator = SatelliteCommunicator(port="/dev/ttyAMA0", baudrate=9600)
        self.mision_is_on = True
        self.autonomy_is_on = False
    def loop_mission(self):
        while self.mision_is_on:
            self.satellite_communicator.read_message()
            self.run_stage_pipeline(self.satellite_communicator.current_stage)

    def run_stage_pipeline(self, stage):
        # Define actions based on the current stage
        if stage == 1:
            self.handle_stage_1()
        elif stage == 2:
            self.handle_stage_2()
        elif stage == 3:
            self.handle_stage_3()
        # Add more stages as needed

    def handle_stage_1(self):
        print("Executing tasks for Stage 1...")
        if self.satellite_communicator.arm_status == True:
            self.stm_com




    def handle_stage_2(self):
        print("Executing tasks for Stage 2...")
        # Add specific tasks or operations for Stage 2

    def handle_stage_3(self):
        print("Executing tasks for Stage 3...")
        # Add specific tasks or operations for Stage 3