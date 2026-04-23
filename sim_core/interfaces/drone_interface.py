from typing import List
import numpy as np
from sensor_interface import ISensor
from world_interface import IWorld

class IDrone():
    name: str
    world: IWorld
    sensor_array: List[ISensor]
    current_position: np.ndarray
    current_heading: np.ndarray
    
    def update_current_data(self):
        pass
    def update_position(self,long,lat,heading,depth = None):
        pass
    def __init__(self):
        pass