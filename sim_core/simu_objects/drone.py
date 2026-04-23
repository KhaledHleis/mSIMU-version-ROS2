from typing import List
import numpy as np

from sim_core.metaclasses.simu_class import SIMU

from world import World

from sim_core.interfaces.sensor_interface import ISensor
from sim_core.interfaces.drone_interface import IDrone
from sim_core.interfaces.world_interface import IWorld
from clock import Clock


class Drone(SIMU,IDrone):
    name: str
    world: IWorld
    sensor_array: List[ISensor]
    current_position: np.ndarray
    current_heading: np.ndarray
    clock: Clock

    def update_current_data(self):
        for sensor in self.sensor_array:
            c = sensor.make_measurement(self)
    def update_position(self, long, lat, heading, depth=None):
        self.current_heading = heading
        self.current_position = np.array(
            [[long, lat, self.current_position[0, 2] if depth is None else depth]]
        )

    def __init__(self, name):
        super().__init__(name)
        self.clock = Clock()
