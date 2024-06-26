import logging
import math
import numpy as np
import time

from src.config import config
from src.constants import Gear
from src.driving.can import ICANController
from src.driving.speed_controller import ISpeedController, SpeedControllerState


from src.utils.lidar import BaseLidar


class ParkingHandler():
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

    def wait_for_wall(self) -> None:
        """Wait until the lidar detects a wall."""
        while self.__lidar.free_range(
                config["parking"]["side_angles"]["right"][0],
                config["parking"]["side_angles"]["right"][1],
                config["parking"]["free_side"]["max_distance"],
        ):
            time.sleep(0.1)

        self.wait_for_opening()

    def wait_for_opening(self) -> None:
        """Wait for the opening to be reached."""
        amount = 0
        while self.__lidar.free_range(
                config["parking"]["side_angles"]["right"][0],
                config["parking"]["wait_for_opening"]["max_angle"],
                config["parking"]["free_side"]["max_distance"],
        ):
            time.sleep(0.1)
            if amount == 3:
                break
            amount += 1

        if amount < 3:
            time.sleep(0.1)
            return self.wait_for_opening()

        return self.wait_for_steering_point()

    def wait_for_steering_point(self) -> None:
        """Wait for the steering point to be reached."""
        while True:
            time.sleep(0.1)
            # Check if we are still detecting a wall.
            if not self.__lidar.free_range(280, 320, config["parking"]["free_side"]["max_distance"]):
                # Get the distance to the wall nad the right most point of it and calculate the distance to it.
                distance_wall = self.__lidar.find_obstacle_distance(265, 280)
                rightmost_point = self.__lidar.find_rightmost_point(
                    280, config["parking"]["rightmost_point_max"], 400, config["parking"]["free_side"]["max_distance"]
                )
                if distance_wall == 0 or rightmost_point == 0:
                    continue

                if rightmost_point < distance_wall or rightmost_point > 10000:
                    continue

                distance_to_spot = math.sqrt((rightmost_point ** 2) - (distance_wall ** 2))

                # If the kart is close enough to the spot steer the kart.
                if distance_to_spot >= config["parking"]["kart_length"]:
                    # Stop and steer the kart
                    self.__speed_controller.target_speed = 0
                    self.__speed_controller.toggle()
                    time.sleep(2)
                    self.__can_controller.set_steering(config["parking"]["steering_angle"])
                    time.sleep(1)
                    # Drive backwards into the parking spot.
                    self.__speed_controller.toggle()
                    self.__speed_controller.gear = Gear.REVERSE
                    self.__speed_controller.target_speed = config["parking"]["speed"]
                    self.drive_into_spot()
                    continue

    def drive_into_spot(self) -> None:
        """Drive into the parking spot."""
        counter = 0
        corner_angle = self.__lidar.find_nearest_angle(250, 290)
        corner = self.__lidar.scan_data[corner_angle]
        wall_2 = self.__lidar.find_highest_index(corner_angle, 320, corner, 8000)
        wall_2_distance = self.__lidar.scan_data[wall_2]
        initial_distance = wall_2_distance - corner
        while True:
            time.sleep(0.1)
            corner_angle = self.__lidar.find_nearest_angle(250, 290)
            corner = self.__lidar.scan_data[corner_angle]
            wall_1 = self.__lidar.find_lowest_index(200, corner_angle, corner, 8000)
            wall_2 = self.__lidar.find_highest_index(corner_angle, 320, corner, 8000)
            indices = np.where(self.__lidar.scan_data[corner_angle:320] == np.inf)[0]
            wall_1_distance = self.__lidar.scan_data[wall_1]
            wall_2_distance = self.__lidar.scan_data[wall_2]

            if len(indices) > 0:
                wall_2 = corner_angle + indices[0]

            if wall_1 == 0 or wall_2 == 0:
                continue

            if wall_2_distance - corner > initial_distance + 1000:
                if counter == 3:
                    self.__can_controller.set_steering(0)
                    return self.wait_to_steer_back()
                counter += 1

            if (
                    corner_angle - wall_1 < 29
                    and wall_2 - corner_angle > 4
                    and (wall_2_distance - corner) > 50
                    and (wall_1_distance - corner) > 50
            ):
                if counter == 3:
                    self.__can_controller.set_steering(-config["parking"]["steering_angle"])
                    return self.wait_to_stop()
                counter += 1

    def wait_to_steer_back(self) -> None:
        """Wait for the go-kart to steer back."""
        counter = 0
        while True:
            time.sleep(0.1)
            corner_angle = self.__lidar.find_nearest_angle(180, 320)

            if corner_angle < config["parking"]["corner_angle"]:
                if counter == 3:
                    self.__can_controller.set_steering(-config["parking"]["steering_angle"])
                    return self.wait_to_stop()
                counter += 1

    def wait_to_stop(self) -> None:
        """Wait till the go-kart almost crosses the line."""
        counter = 0
        while True:
            time.sleep(0.1)
            lowest_angle = self.__lidar.find_lowest_index(0, 150, 300, 9000)
            highest_angle = self.__lidar.find_highest_index(180, 320, 300, 9000)
            angle = highest_angle - lowest_angle

            left_wall = self.__lidar.find_lowest_index(120, 250, 300, 9000)
            right_wall = self.__lidar.find_highest_index(160, 310, 300, 9000)
            deviation = (right_wall + left_wall) / 2 - 180

            if 10 < deviation < 10:
                self.__speed_controller.target_speed = 0
                self.__speed_controller.state = SpeedControllerState.STOPPED
                self.__can_controller.set_steering(0)
                return None

            if angle < config["parking"]["angle_threshold"]:
                self.__speed_controller.target_speed = 0
                self.__speed_controller.toggle()
                self.__can_controller.set_steering(0)
                return self.forward_creep(False)
            if lowest_angle == 0:
                if counter == 2:
                    self.__speed_controller.target_speed = 0
                    self.__speed_controller.toggle()
                    logging.error("Cannot see the left wall, failed parking.")
                    return None

                counter += 1
            time.sleep(0.1)

    def forward_creep(self, reverse: bool) -> None:
        """Align with the parking spot by driving forward."""
        counter = 0
        while True:
            time.sleep(0.1)

            left_wall = self.__lidar.find_lowest_index(120, 250, 300, 9000)
            right_wall = self.__lidar.find_highest_index(160, 310, 300, 9000)
            self.__speed_controller.gear = Gear.DRIVE
            self.__speed_controller.target_speed = config["parking"]["speed"]

            if right_wall == 0:
                continue
            deviation = (right_wall + left_wall) / 2 - 180

            if deviation > 2:
                self.__can_controller.set_steering(config["parking"]["steering_angle"])
            else:
                self.__can_controller.set_steering(-config["parking"]["steering_angle"])

            if -15 < deviation < 15:
                self.__speed_controller.state = SpeedControllerState.STOPPED
                return

            if not self.__lidar.free_range(140, 250, 800) and reverse:
                self.reverse_creep(reverse)
                counter += 1
                return

            if self.__lidar.free_range(160, 220, 2500):
                reverse = False

            self.__speed_controller.gear = Gear.DRIVE
            self.__speed_controller.target_speed = config["parking"]["speed"]
            if self.__lidar.free_range(160, 220, 500):
                reverse = True

    def reverse_creep(self, reverse: bool) -> None:
        """Reverse the go-kart to align it with the parking spot."""
        counter = 0
        while True:
            time.sleep(0.1)
            left_wall = self.__lidar.find_lowest_index(120, 250, 300, 9000)
            right_wall = self.__lidar.find_highest_index(150, 310, 300, 9000)
            deviation = (right_wall + left_wall) / 2 - 180

            lowest_angle = self.__lidar.find_lowest_index(30, 150, 700, 9000)
            highest_angle = self.__lidar.find_highest_index(180, 320, 700, 9000)
            angle = highest_angle - lowest_angle

            if angle < 233 and deviation > 40:
                self.__speed_controller.target_speed = config["parking"]["speed"]
                self.__can_controller.set_steering(0)
                return self.forward_creep(False)
            # If the deviation is in the left direction.
            if deviation < 0:
                self.forward_creep(reverse)
                self.__can_controller.set_steering(config["parking"]["steering_angle"])
            else:
                self.__can_controller.set_steering(-config["parking"]["steering_angle"])
            # If the car is aligned with the parking spot, stop the car.
            if -15 < deviation < 15:
                self.__speed_controller.target_speed = config["parking"]["speed"]
                if self.__lidar.free_range(160, 250, 800):
                    self.forward_creep(reverse)

                self.__speed_controller.state = SpeedControllerState.STOPPED
                return None

            if not reverse:
                self.forward_creep(reverse)
            elif counter > 3:
                reverse = True

            if self.__lidar.free_range(120, 240, 2000):
                reverse = False
                counter = 0
                self.forward_creep(reverse)
            else:
                self.__speed_controller.gear = Gear.REVERSE
                self.__speed_controller.target_speed = config["parking"]["speed"]
            time.sleep(0.1)
