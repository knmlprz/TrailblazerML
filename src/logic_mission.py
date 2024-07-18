from logic_mission.satellite_communicator import SatelliteCommunicator
from communication.stm_com import STMCom
from go_autonomy import GoAutonomy
import time

class Mission:
    def __init__(self):
        self.stm_com = STMCom(port="/dev/ttyACM0")
        self.satellite_communicator = SatelliteCommunicator(port="/dev/ttyTHS1", baudrate=115200)
        self.mision_is_on = True
        self.autonomy_is_on = False
        self.stm_com.led_r = True
        self.stm_com.led_g = False
        self.stm_com.led_y = False
        self.stm_com.girpper_open = False
        self.stm_com.send_command()

    def loop_mission(self):
        while self.mision_is_on:
            self.satellite_communicator.read_message()
            self.run_stage_pipeline(self.satellite_communicator.current_stage)

    def run_stage_pipeline(self, stage):
        # Define actions based on the current stage
        if self.satellite_communicator.arm_status:

            if stage == 1:
                self.handle_stage_1()
            elif stage == 2:
                self.handle_stage_2()
            elif stage == 3:
                self.handle_stage_3()
            # Add more stages as needed
        else:
            self.mision_is_on = False

    def handle_stage_1(self):
        print("Executing tasks for Stage 1...")
        time.sleep(3)
        if self.satellite_communicator.arm_status != 0:
            time.sleep(3)
            self.stm_com.led_r = False
            self.stm_com.led_g = True
            self.stm_com.led_y = False
            self.stm_com.girpper_open = False
            self.stm_com.send_command()

            if self.satellite_communicator.latitude != None and self.satellite_communicator.longitude != None:
                time.sleep(3)
                self.stm_com.led_r = False
                self.stm_com.led_g = False
                self.stm_com.led_y = True
                self.stm_com.girpper_open = False
                self.stm_com.send_command()
                go_autonomy = GoAutonomy( self.stm_com, self.satellite_communicator)
                while self.satellite_communicator.arm_status != 0 and not go_autonomy.rover_in_target:
                    go_autonomy.run()

                self.stm_com.led_r = False
                self.stm_com.led_g = False
                self.stm_com.led_y = True
                self.stm_com.girpper_open = True
                self.stm_com.send_command()

                self.stm_com.led_r = False
                self.stm_com.led_g = True
                self.stm_com.led_y = False
                self.stm_com.girpper_open = False
                self.stm_com.send_command()

                self.satellite_communicator.task_completed()

    def handle_stage_2(self):
        print("Executing tasks for Stage 2...")
        # Add specific tasks or operations for Stage 2

    def handle_stage_3(self):
        print("Executing tasks for Stage 3...")
        # Add specific tasks or operations for Stage 3

if __name__ == "__main__":
    mission = Mission()
    mission.loop_mission()