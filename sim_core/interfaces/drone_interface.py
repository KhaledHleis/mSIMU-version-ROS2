from typing import List
import numpy as np

from sim_core.interfaces.sensor_interface import ISensor
from sim_core.interfaces.world_interface import IWorld

class IDrone():
    name: str
    world: IWorld
    sensor_array: List[ISensor]
    current_position: np.ndarray
    current_rotation: np.ndarray # euler angles in radians, order: roll, pitch, yaw
    
    def update_current_data(self):
        pass
    def update_position(self,long,lat,rotation,depth = None):
        pass
    def __init__(self):
        pass