from typing import Tuple, Optional
import numpy as np

from metaclasses.simu_class import SIMU

from interfaces.drone_interface import IDrone
from interfaces.world_interface import IWorld
from interfaces.sensor_interface import ISensor
from interfaces.target_interface import ITarget
from simu_objects.clock import Clock

from utils.utilities_initiliser import *

import time


class Experiment(SIMU):
    
    world: IWorld
    drone: IDrone
    clock : Clock
    
    def run(self,input_path):

        #! set drone in place
        self.drone.update_position(
            longitude_array[0], latitude_array[0], heading_array[0], depth=0
        )
        #! initiate loggers
        if(not self.skip_logging):
            drone_logger, world_logger, self.name = initialize_loggers_batch_with_timestamp(
                self.name, batch_size=10000,flush_frequency=0.001
            )
        #! program loop over all trajectory points
        if(not self.skip_logging): world_logger.log(self.world)
        print("experiment >>>>> number of trajectory points ", len(longitude_array))
        for longitude, latitude, heading in zip(
            longitude_array, latitude_array, heading_array
        ):
            self.drone.update_position(longitude, latitude, heading)
            self.drone.update_current_data()
            clock.set_conversion_factor(delta_timestamp)
            clock.increment_time()
            if(not self.skip_logging):
                drone_logger.log(self.drone)
        print("experiment >>>>> experiment ended saving in progress ...")
        drone_logger.wait_until_complete()

    
    
    def __init__(self,config_obj):
        super().__init__(config_obj["experiment"]["name"])
        #! initialise all from configuration object        
        self.world,self.drone = initialise_ALL(config_obj)
        #! activate the clock
        self.clock = Clock()



