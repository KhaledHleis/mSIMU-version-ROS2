from typing import List
import numpy as np

from sim_core.metaclasses.simu_class import SIMU


from sim_core.simu_objects.clock import Clock

from sim_core.interfaces.world_interface import IWorld
from sim_core.interfaces.sensor_interface import ISensor
from sim_core.interfaces.drone_interface import IDrone

from sim_core.utils.utilities_converter import lld_to_ned

class Drone(SIMU,IDrone):
    name: str
    sensor_array: List[ISensor]
    current_position: np.ndarray
    current_rotation: np.ndarray
    clock: Clock
    world: IWorld

    def update_current_data(self):
        for sensor in self.sensor_array:
            c = sensor.make_measurement(self)

    def update_position(self, long, lat, rotation, depth=None):
        self.current_rotation = rotation
        self.current_position = lld_to_ned( np.array(
            [[long, lat, self.current_position[0, 2] if depth is None else depth]]
        ),self.world.reference_point)

    def __init__(self, name,sensor_array: List[ISensor],world: IWorld):
        super().__init__(name)
        self.sensor_array = sensor_array
        self.clock = Clock()
        self.world = world
