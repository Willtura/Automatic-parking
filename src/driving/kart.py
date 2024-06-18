import time

from typing import Any

from src.driving.can import CANController, get_can_bus
from src.driving.modes import AutonomousDriving, DrivingMode, ManualDriving


class Kart:
    """The main class for the system. This class will start the kart and all its components.

    Attributes
    ----------
        autonomous (AutonomousDriving): The autonomous driving mode.
        manual (BasicControllerDriving): The manual driving mode.

    """

    autonomous: AutonomousDriving
    manual: DrivingMode

    __autonomous: bool = False
    __can: CANController

    def __init__(self) -> None:
        """Initialize the kart."""
        bus = get_can_bus()
        self.__can = CANController(bus)

        self.autonomous = AutonomousDriving(self.__can)
        self.manual = ManualDriving(self.__gamepad, self.__can)

        self.autonomous.telemetry.add_callback_function("toggle_driving_mode", self.__toggle)

    def start(self) -> "Kart":
        """Start the kart and all its components."""
        self.__can.start()

        self.manual.start()
        self.autonomous.start()

        return self


