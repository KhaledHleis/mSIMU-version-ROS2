from abc import abstractmethod
import numpy as np

from sim_core.metaclasses.simu_class import SIMU
from sim_core.interfaces.target_interface import ITarget

class Target(SIMU,ITarget):
    
    @abstractmethod
    def calculate_field_at_position(self,position:np.ndarray)->np.ndarray:
        """
        
        An instance function for the target class that calculates the magnetic field vector from a point position in 3D space

        Args:
            position (np.ndarray): absolute sensor position

        Returns:
            np.ndarray: magnetic field vector or shape (1,3)
        """
        pass
    

class Dipole(Target):
    
    def calculate_field_at_position(self, position) -> np.ndarray:
        position = np.asarray(position, dtype=float)

        R = position - self.center_point
        r = np.linalg.norm(R)

        if r == 0.0:
            raise ValueError("Field is undefined at the dipole center")

        R_hat = R / r
        m = self.dipole_moment

        field = (3.0 * np.dot(m, R_hat) * R_hat - m) / (r ** 3)
        return field

    def __init__(self, name:str, center_point:np.ndarray,dipole_moment:np.ndarray):
        self.name = name
        self.center_point = center_point
        self.dipole_moment = dipole_moment
        
        import numpy as np


class Cable(Target):

    def __init__(
        self,
        name: str,
        start_point: np.ndarray,
        end_point: np.ndarray,
        current: float,
        current_freq: float # WORK implement frequency
    ):
        self.name = name
        self.start_point = np.asarray(start_point, dtype=float)
        self.end_point = np.asarray(end_point, dtype=float)
        self.current = current

    def calculate_field_at_position(self, position) -> np.ndarray:
        position = np.asarray(position, dtype=float)

        R1 = position - self.start_point
        R2 = position - self.end_point
        L = self.end_point - self.start_point

        r1 = np.linalg.norm(R1)
        r2 = np.linalg.norm(R2)

        cross_R1_R2 = np.cross(R1, R2)
        cross_norm_sq = cross_R1_R2 @ cross_R1_R2.T

        if cross_norm_sq == 0.0:
            raise ValueError("Field is undefined on the cable axis")

        term = (R1@L.T / r1) - (R2@L.T / r2)

        # μ₀/(4π) = 10^-7 T·m/A = 100 nT·m/A
        # Use 100 if you want output in nanoTesla
        mu_0_over_4pi = 100  # nT·m/A
        
        field = mu_0_over_4pi * self.current * cross_R1_R2 / cross_norm_sq * term
        return field