from logic_mission.satellite_communicator import SatelliteCommunicator
from communication.stm_com import STMCom
from navigate_on_gps import RoverController
import time


class LogicMission:
    """
    Class for the creation logic of the mission. It will handle the stages of the mission and the actions to be taken in each stage.
    """

    def __init__(self):

        self.stm_com = STMCom(port="/dev/ttyACM0")
        self.satellite_communicator = SatelliteCommunicator(
            port="/dev/ttyTHS1", baudrate=115200
        )
        self.mision_is_on = True
        self.autonomy_is_on = False

    def loop_mission(self):
        """
        Loop the mission until the mission is finished or special flag (mision_is_on)  is set to False.
        Returns:

        """
        while self.mision_is_on:
            self.satellite_communicator.read_message()
            self.run_stage_pipeline(self.satellite_communicator.current_stage)

    def run_stage_pipeline(self, stage: SatelliteCommunicator.current_stage):
        """
        Run the pipeline of the mission stages. The stages are received by the satellite communicator.
        Args:
            stage:  (SatelliteCommunicator.current_stage) The current stage of the mission.

        Returns:

        """
        if self.satellite_communicator.arm_status:

            if stage == 1:
                self.handle_stage_1()
            elif stage == 2:
                self.handle_stage_2()
            elif stage == 3:
                self.handle_stage_3()
        else:
            self.mision_is_on = False

    def handle_stage_1(self):
        """
        Handle the tasks for Stage 1 of the mission.
        1. Check if the arm is ready.
        2. Check if the GPS coordinates are ready.
        3. Navigate to the GPS coordinates.
        4. Open the gripper.
        5. Send the task completed message to the satellite.
        Returns:

        """
        time.sleep(0.5)
        self.stm_com.led_r = True
        self.stm_com.led_g = False
        self.stm_com.led_y = False
        self.stm_com.girpper_open = False
        self.stm_com.send_command()
        print("Executing tasks for Stage 1...")
        time.sleep(0.5)
        if self.satellite_communicator.arm_status != 0:
            time.sleep(0.5)
            self.stm_com.led_r = False
            self.stm_com.led_g = True
            self.stm_com.led_y = False
            self.stm_com.girpper_open = False
            self.stm_com.send_command()

            if (
                self.satellite_communicator.latitude != None
                and self.satellite_communicator.longitude != None
            ):
                time.sleep(0.5)
                self.stm_com.led_r = False
                self.stm_com.led_g = False
                self.stm_com.led_y = True
                self.stm_com.girpper_open = False
                self.stm_com.send_command()
                rc = RoverController("utils/position.json", 39.8849380, 32.7775717)
                while self.satellite_communicator.arm_status != 0 or not rc.on_goalt:
                    rc.navigate()

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
    mission = LogicMission()
    mission.loop_mission()
