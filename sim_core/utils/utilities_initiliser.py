import numpy as np

from sim_core.simu_objects.target import Target,Cable,Dipole
from sim_core.simu_objects.sensor import Sensor,Fluxgate,Scalar
from sim_core.simu_objects.world import World
from sim_core.simu_objects.drone import Drone

from typing import List

from sim_core.utils.utilities_converter import *

# VERIF that all initialisers are working correctly
# region helper functions

def __initialise_targets (config_object,ref_lld)-> List[Target]:
    Target_array = []
    # import cables
    if("cables" in config_object):
        for target_object in config_object["cables"]:
            
            name = target_object["name"]
            start_long = target_object["starting_longitude"]
            start_lat = target_object["starting_latitude"]
            start_depth = target_object["starting_depth"]
            end_long = target_object["ending_longitude"]
            end_lat = target_object["ending_latitude"]
            end_depth = target_object["ending_depth"]
            current = target_object["current"]
            freq = target_object["current_frequency"] 
            
            print("<"*5,"Initialising cable with start long:", start_long, "and end point:", end_long, ">"*5)
            start_point = lld_to_ned(np.array([[start_long,start_lat,start_depth]]),ref_lld)
            end_point = lld_to_ned(np.array([[end_long,end_lat,end_depth]]),ref_lld)
            print("<"*5,"Initialising cable with start point:", start_point, "and end point:", end_point, ">"*5)
            Target_array.append(Cable(name,start_point,end_point,current,freq))
    # import dipoles
    if("dipoles" in config_object):
        for target_object in config_object["dipoles"]:
            
            name = target_object["name"]
            center_long = target_object["center_longitude"]
            center_lat = target_object["center_latitude"]
            center_depth = target_object["center_depth"]
            moment = np.array(target_object["dipole_moment"])
            
            center_point = lld_to_ned(np.array([[center_long,center_lat,center_depth]]),ref_lld)
            
            Target_array.append(Dipole(name,center_point,moment))
        
    return Target_array

def __initialise_sensors (config_object)-> List[Sensor]:
    sensor_array = []
    for sensor in config_object["sensors"]:
        name = sensor["name"]
        sensor_type = sensor["type"]
        relative_pos = np.array([sensor["relative_position"]])
        
        if(sensor_type == "Fluxgate"):
            sensor_array.append(Fluxgate(name,relative_pos))
        elif(sensor_type == "Scalar"):
            sensor_array.append(Scalar(name,relative_pos))
        else:
            raise TypeError("sensor type does not exist")
    return sensor_array

def __initialise_word (config_object)->World:
    config_world = config_object["world"]
    name = config_world["name"]
    ref_long = config_world["reference_longitude"]
    ref_lat = config_world["reference_latitude"]
    ref_depth = config_world["reference_depth"]
    simulation_radius = config_world["simulation_radius"]
    regional_field = np.array(config_world["regional_magnetic_field"])
    
    ref_lld = np.array([[ref_long,ref_lat,ref_depth]])
    target_array = __initialise_targets(config_object,ref_lld)
    
    return World(name,target_array,simulation_radius,regional_field)

def __initialise_drone (config_object)->Drone:
    config_drone = config_object["drone"]
    name = config_drone["name"]
    if("sensors" not in config_object):
        raise TypeError("a drone must have sensors")
    sensor_array = __initialise_sensors(config_object)
    return Drone(name,sensor_array,__initialise_word(config_object))

# endregion

def initialise_ALL (config_object)->tuple[World,Drone]:
    drone = __initialise_drone(config_object)
    world = drone.world
    return world,drone