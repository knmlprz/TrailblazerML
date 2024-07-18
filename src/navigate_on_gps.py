import json
import math
import time
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

from communication.stm_com import STMCom


class RoverController:
    def __init__(self, gps_file, latitude_goal, longitude_goal,stm_com, tolerance=0.0001,):
        self.stm_com =stm_com
        self.gps_file = gps_file
        self.latitude_goal = latitude_goal
        self.longitude_goal = longitude_goal
        self.tolerance = tolerance
        self.latitude_current = None
        self.longitude_current = None
        self.direction = None
        self.on_goal =False
        self.fuzzy_controller = self.create_fuzzy_controller()
        self.path = []  # Lista do przechowywania trasy

    def create_fuzzy_controller(self):
        # Definicja zmiennych wejściowych i wyjściowych
        #
        #
        angle = ctrl.Antecedent(np.arange(-180, 181, 1), 'angle')
        angle['left'] = fuzz.trapmf(angle.universe, [-180, -180, -90, 0])
        angle['straight'] = fuzz.trimf(angle.universe, [-10, 0, 10])
        angle['right'] = fuzz.trapmf(angle.universe, [0, 90, 180, 180])

        left_wheel_speed = ctrl.Consequent(np.arange(0, 101, 1), 'left_wheel_speed')
        right_wheel_speed = ctrl.Consequent(np.arange(0, 101, 1), 'right_wheel_speed')

        left_wheel_speed['slow'] = fuzz.trapmf(left_wheel_speed.universe, [0, 0, 30, 50])
        left_wheel_speed['medium'] = fuzz.trimf(left_wheel_speed.universe, [30, 50, 70])
        left_wheel_speed['fast'] = fuzz.trapmf(left_wheel_speed.universe, [50, 70, 100, 100])

        right_wheel_speed['slow'] = fuzz.trapmf(right_wheel_speed.universe, [0, 0, 30, 50])
        right_wheel_speed['medium'] = fuzz.trimf(right_wheel_speed.universe, [30, 50, 70])
        right_wheel_speed['fast'] = fuzz.trapmf(right_wheel_speed.universe, [50, 70, 100, 100])

        # Definicja reguł logiki rozmytej
        rule1 = ctrl.Rule(angle['left'], (left_wheel_speed['slow'], right_wheel_speed['fast']))
        rule2 = ctrl.Rule(angle['straight'], (left_wheel_speed['fast'], right_wheel_speed['fast']))
        rule3 = ctrl.Rule(angle['right'], (left_wheel_speed['fast'], right_wheel_speed['slow']))

        # Tworzenie systemu kontrolnego
        speed_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])
        return ctrl.ControlSystemSimulation(speed_ctrl)

    def read_gps_data(self):
        with open(self.gps_file, 'r') as file:
            data = json.load(file)
            self.latitude_current = data[-1]["latitude"]
            self.longitude_current = data[-1]["longitude"]
            self.path.append((self.latitude_current, self.longitude_current))  # Dodaj aktualną pozycję do trasy

    def calculate_direction(self):
        delta_latitude = self.latitude_goal - self.latitude_current
        delta_longitude = self.longitude_goal - self.longitude_current
        self.direction = math.degrees(math.atan2(delta_longitude, delta_latitude))

    def calculate_distance(self):
        return math.sqrt(
            (self.latitude_goal - self.latitude_current) ** 2 + (self.longitude_goal - self.longitude_current) ** 2)

    def control_rover(self):
        self.fuzzy_controller.input['angle'] = self.direction
        self.fuzzy_controller.compute()
        left_speed = self.fuzzy_controller.output['left_wheel_speed']
        right_speed = self.fuzzy_controller.output['right_wheel_speed']
        self.move_rover(left_speed, right_speed)

    def move_rover(self, left_speed, right_speed):
        self.stm_com.update(left_speed,right_speed)
        # Kod do sterowania łazikiem z wykorzystaniem prędkości kół
        print(f"Moving with left wheel speed: {left_speed} and right wheel speed: {right_speed}")

    def navigate(self):
            self.read_gps_data()
            distance_to_goal = self.calculate_distance()
            if distance_to_goal <= self.tolerance:
                print("done!!!!")
                self.on_goal =True
            self.calculate_direction()
            self.control_rover()
            time.sleep(1)

    # def plot_path(self):
    #     latitudes, longitudes = zip(*self.path)
    #     plt.plot(longitudes, latitudes, marker='o')
    #     plt.plot(self.longitude_goal, self.latitude_goal, marker='x', markersize=12, color='red')
    #     plt.xlabel('Longitude')
    #     plt.ylabel('Latitude')
    #     plt.title('Rover Path')
    #     plt.grid(True)
    #     plt.show()


# Przykład użycia klasy RoverController

