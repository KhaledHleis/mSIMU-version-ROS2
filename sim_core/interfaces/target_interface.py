from abc import abstractmethod
import numpy as np

class ITarget():
    @abstractmethod
    def calculate_field_at_position(self,position:np.ndarray)->np.ndarray:
        pass
    def __init__(self):
        pass