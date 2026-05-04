from typing import Tuple, Optional
import numpy as np

from sim_core.metaclasses.simu_class import SIMU

from sim_core.interfaces.drone_interface import IDrone
from sim_core.interfaces.world_interface import IWorld
from sim_core.interfaces.sensor_interface import ISensor
from sim_core.interfaces.target_interface import ITarget
from sim_core.simu_objects.clock import Clock

from sim_core.utils.utilities_initiliser import *
from sim_core.utils.utilities_converter import ned_to_lld

class Experiment(SIMU):
    
    world: IWorld
    drone: IDrone
    clock : Clock
    last_timestamp:int = 0
    
    def update(self, input_point, timestamp):        
        self.clock.set_delta_t(timestamp - self.last_timestamp)
        
        long, lat, depth, roll, pitch, yaw = input_point            
        rotation = np.array([[roll, pitch, yaw]])
        self.drone.update_position(long, lat, rotation, depth=depth)
        self.drone.update_current_data()
        
        self.clock.set_time(timestamp)
        self.last_timestamp = timestamp

    def get_measurements(self):  
        measurments_object = {}      
        for sensor in self.drone.sensor_array:
            # WORK: we can add the timestamp to the measurement object if needed, for now we will just return the measurements
            measurments_object[sensor.name] = sensor.measurement
        return measurments_object
    
    def batch_measurements_and_updates(self, input_path, time_array, out_array=False):
        """
        Args:
            out_array (bool, optional): output an array of measurments (array, sensor_names) instead of a list of objects. Defaults to False.
        """
        measurement_array = []
        for input_point, timestamp in zip(input_path, time_array):
            
            self.update(input_point, timestamp)
            measurement = self.get_measurements()
            measurement_array.append(measurement)
            
        if out_array:
            result, sensors = transform_to_mn3(measurement_array)
            return result, sensors
        else:
            return measurement_array
    
    def batch_CSV_updates(self, input_path, time_array):
        csv_row = []
        csv_row.append(["timestamp", "longitude", "latitude", "depth", "heading", "magx", "magy", "magz","mag"])
        
        for input_point, timestamp in zip(input_path, time_array):
            self.update(input_point, timestamp)
            long, lat,depth = ned_to_lld(self.drone.current_position,self.world.reference_point).flatten()
            heading = self.drone.current_rotation.flatten()[2]
            magx, magy, magz = self.drone.sensor_array[0].measurement.flatten()
            mag = np.linalg.norm([magx, magy, magz])
            csv_row.append([timestamp, long, lat, depth, heading, magx, magy, magz, mag])
        return csv_row

    def __init__(self,config_obj):
        super().__init__(config_obj["experiment"]["name"])
        #! initialise all from configuration object        
        self.world,self.drone = initialise_ALL(config_obj)
        #! activate the clock
        self.clock = Clock()



