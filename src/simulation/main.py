import airsim

from src.config import config
from src.constants import Gear
from src.driving.speed_controller import SpeedController, SpeedControllerState

from src.parking.parking import ParkingHandler
from src.parking.parking_forward import Forward_ParkingHandler
from src.simulation.can_controller import SimCanController
from src.simulation.sim_lidar import SimLidar


def start_simulator() -> None:
    """Run the simulator."""

    client = airsim.CarClient()
    client.confirmConnection()

    can_controller = SimCanController()
    speed_controller = SpeedController(can_controller)
    speed_controller.gear = Gear.DRIVE
    speed_controller.state = SpeedControllerState.DRIVING
    speed_controller.max_speed = 5

    lidar = SimLidar(client)
    lidar.start()

    parking_handler = ParkingHandler(speed_controller, lidar)

    forward_parkinghandler = Forward_ParkingHandler(speed_controller, lidar)
    speed_controller.toggle()

    speed_controller.state = SpeedControllerState.PARKING
    speed_controller.target_speed = config["parking"]["speed"]

    parking_handler.wait_for_wall()

    #forward_parkinghandler.forward_park()
