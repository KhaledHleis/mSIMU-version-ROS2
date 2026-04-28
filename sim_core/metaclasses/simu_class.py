from abc import ABC,abstractmethod
from sim_core.metaclasses.string_convertable  import StringConvertible

class SIMU(ABC,StringConvertible):
    
    @abstractmethod
    def __init__(self,name:str):
        self.name = name