# ? python packages
import numpy as np
from abc import abstractmethod

# ? abstract classes
from sim_core.metaclasses.simu_class import SIMU

# ? utilities
from sim_core.utils.utilities_converter import (
    body_to_ned,
    ned_to_body,
)

# ? interfaces
from sim_core.interfaces.sensor_interface import ISensor
from sim_core.interfaces.drone_interface import IDrone


class Sensor(SIMU, ISensor):
    name: str
    relative_position: np.ndarray

    @abstractmethod
    def make_measurement(self, parent_drone: IDrone)-> np.ndarray:
        pass

    def __init__(self, name, relative_position: np.ndarray):
        super().__init__(name)
        self.relative_position = relative_position


class Fluxgate(Sensor):

    def make_measurement(self, parent_drone):
        world = parent_drone.world

        # Get field in NED frame at sensor position
        field_ned = world.calculate_entire_field_at_position(
            body_to_ned(
                parent_drone.current_position.flatten(),
                parent_drone.current_rotation[0, 0],    # roll
                parent_drone.current_rotation[0, 1],    # pitch
                parent_drone.current_rotation[0, 2],     # yaw
                self.relative_position.flatten(),
            )
        )

        # Convert from NED to body frame using drone's attitude
        field_body = ned_to_body(
            field_ned.flatten(), 
            parent_drone.current_rotation[0, 0],    # roll
            parent_drone.current_rotation[0, 1],    # pitch
            parent_drone.current_rotation[0, 2]     # yaw
        )
        # print(">>> field measurement : ", field_body)
        self.measurement = field_body
        return field_body

class Scalar(Sensor):

    def make_measurement(self, parent_drone):
        world = parent_drone.world

        # Get field in NED frame at sensor position
        field_ned = world.calculate_entire_field_at_position(
            body_to_ned(
                parent_drone.current_position.flatten(),
                parent_drone.current_rotation[0, 0],    # roll
                parent_drone.current_rotation[0, 1],    # pitch
                parent_drone.current_rotation[0, 2],     # yaw
                self.relative_position.flatten(),
            )
        )

        # Convert from NED to body frame using drone's attitude
        field_body = ned_to_body(
            field_ned.flatten(), 
            parent_drone.current_rotation[0, 0],    # roll
            parent_drone.current_rotation[0, 1],    # pitch
            parent_drone.current_rotation[0, 2]     # yaw
        )
        self.measurement = np.linalg.norm(field_body)
        return self.measurement
