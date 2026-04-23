from sim_core.metaclasses.singleton import Singleton


class Clock(metaclass=Singleton):
    """
    Singleton clock class.

    Keeps a global timestamp and applies a conversion factor
    when incrementing time.

    timestamp is an integer in [ms] milliseconds,
    and the conversion_factor that multiplies the increment value for example it could be the sampling rate x1000.
    """

    def __init__(self):
        self.__time_stamp: int = 0
        self.conversion_factor: int = 1

    def get_time_stamp(self) -> int:
        return self.__time_stamp

    def increment_time(self, delta: int = 1):
        self.__time_stamp += delta * self.conversion_factor

    def set_conversion_factor(self, factor: int):
        self.conversion_factor = factor
