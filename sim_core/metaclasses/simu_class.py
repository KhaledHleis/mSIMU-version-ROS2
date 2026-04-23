from abc import ABC,abstractmethod

class SIMU(ABC):
    
    @abstractmethod
    def __init__(self,name:str):
        self.name = name