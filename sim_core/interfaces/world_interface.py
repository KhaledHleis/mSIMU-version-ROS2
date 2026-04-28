import numpy as np
from typing import List

from abc import abstractmethod
from sim_core.interfaces.target_interface import ITarget

class IWorld():
    reference_point: np.ndarray
    simulation_radius: int
    regional_magnetic_field: np.ndarray
    target_array: List[ITarget]
    
    @abstractmethod
    def calculate_entire_field_at_position(self, position: np.ndarray) -> np.ndarray:
        pass
    
    def __init__(self):
        pass