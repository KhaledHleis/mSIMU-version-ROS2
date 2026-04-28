from sim_core.metaclasses.singleton import Singleton


class Clock(metaclass=Singleton):
    """
    Singleton clock class.

    Keeps a global timestamp and applies a conversion factor
    when incrementing time.

    timestamp is an integer in [ns] nanoseconds,
    """

    def __init__(self):
        self.__time: int = 0

    #! getter and setter for time and delta_t
    def get_time(self) -> int:
        return self.__time

    def set_time(self,time:int):
        self.__time = time # time in ns

    def get_delta_t(self) -> int:
        return self.__delta_t

    def set_delta_t(self, delta_t: int):
        self.__delta_t = delta_t # delta_t in ns