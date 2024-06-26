import logging
import math
import numpy as np
import time

from src.config import config
from src.constants import Gear
from src.driving.can import ICANController
from src.driving.speed_controller import ISpeedController, SpeedControllerState


from src.utils.lidar import BaseLidar


class Forward_ParkingHandler():
    """A handler for parking spaces."""

    __lidar: BaseLidar
    __speed_controller: ISpeedController
    __can_controller: ICANController

    def __init__(self, speed_controller: ISpeedController, lidar: BaseLidar) -> None:
        """Initializes the parking handler.

        :param controller: The controller.
        :param lidar: The lidar sensor.
        """
        self.__lidar = lidar
        self.__speed_controller = speed_controller
        self.__can_controller = speed_controller.can_controller

    def forward_park(self):

        phase = 0
        initial_distance = 0

        while True:
            time.sleep(0.1)
            if phase == 0:
                self.__speed_controller.target_speed = 3
                if not self.__lidar.free_range(260, 280, 2500):
                    distance_wall = self.__lidar.find_obstacle_distance(255, 285)
                    print(distance_wall)
                    phase = 1
                    continue
                print("Wall detected, driving to parking spot.")
                continue

            if phase == 1:
                if self.__lidar.free_range(260, 280, 2500):
                    self.__can_controller.set_steering(1.25)
                    #self.controller.lane_assist.toggle()
                    initial_distance = self.__lidar.find_obstacle_distance(155, 240)
                    if initial_distance == np.inf:
                        continue
                    phase = 2
                    print("Parking spot found, parking.")
                    continue
                time.sleep(0.1)
                continue

            if phase == 2:
                time.sleep(0.1)
                print(initial_distance)
                print(self.__lidar.find_obstacle_distance(80, 180) - initial_distance / 2)
                if self.__lidar.find_obstacle_distance(80, 160) < (initial_distance / 2) + 1700:
                    self.__can_controller.set_steering(-1.25)
                    print("Steering back")
                    phase = 3
                    continue
                continue

            if phase == 3:
                time.sleep(0.1)
                #take the average of the highest and lowest indexes w lidar in front of the car (180) and calculate deviation
                lowest_index = self.__lidar.find_lowest_index(150, 190, 0, 3000)
                highest_index = self.__lidar.find_highest_index(170, 230, 0, 3000)
                deviation = (highest_index + lowest_index) / 2 - 180
                if -10 < deviation < 10:
                    self.__can_controller.set_steering(0)
                    phase = 4
                    continue
                continue

            if phase == 4:
                time.sleep(0.1)
                print(self.__lidar.find_obstacle_distance(160, 200))
                if self.__lidar.find_obstacle_distance(160, 200) < 2200:
                    print("STOP, parking finished.")
                    self.__can_controller.set_steering(0)
                    self.__speed_controller.target_speed = 0
                    self.__speed_controller.toggle()
                    logging.info("Parked, maneuver finished.")
                    continue
                continue

            time.sleep(0.2)