import numpy as np
from typing import List

from sim_core.metaclasses.simu_class import SIMU

from sim_core.interfaces.world_interface import IWorld
from sim_core.interfaces.target_interface import ITarget

from sim_core.simu_objects.clock import Clock
class World(SIMU, IWorld):
    name: str
    reference_point: np.ndarray
    simulation_radius: int
    regional_magnetic_field: np.ndarray
    target_array: List[ITarget]

    def calculate_entire_field_at_position(self, position: np.ndarray) -> np.ndarray:
        B = self.regional_magnetic_field.copy()
        for target in self.target_array:
            B += target.calculate_field_at_position(position)
        return B

    def __init__(self, name, target_array: List[ITarget],simulation_radius:int,regional_magnetic_field:np.ndarray):
        super().__init__(name)
        self.target_array = target_array  # array of all targets in the simulation
        self.simulation_radius = simulation_radius
        self.regional_magnetic_field = np.array(regional_magnetic_field,dtype=float).reshape(1,3)
        self.clock = Clock()