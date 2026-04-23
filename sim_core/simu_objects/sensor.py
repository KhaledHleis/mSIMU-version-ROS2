# ? python packages
import numpy as np
from abc import abstractmethod

# ? abstract classes
from sim_core.metaclasses.simu_class import SIMU

# ? utilities
from sim_core.utils.utilities_converter import (
    Absolute_position,
    convert_field_ned_to_body,
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
            Absolute_position(
                parent_drone.current_position.reshape(-1),
                parent_drone.current_heading.reshape(-1),
                self.relative_position.reshape(-1),
            )
        )

        # Convert from NED to body frame using drone's attitude
        field_body = convert_field_ned_to_body(
            field_ned, 0, 0, parent_drone.current_heading  # yaw
        )
        self.magnetic_field = field_body
        return field_body
