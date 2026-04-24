from typing import Tuple, Optional
import numpy as np

from metaclasses.simu_class import SIMU

from interfaces.drone_interface import IDrone
from interfaces.world_interface import IWorld
from simu_objects.clock import Clock

from backend.simulation.parsers.trajectory_parser import TrajectoryParser
from backend.simulation.parsers.world_parser import WorldParser
from backend.simulation.parsers.drone_parser import DroneParser

from backend.utilities.utilities_logger import initialize_loggers_batch_with_timestamp

import time


class Experiment(SIMU):
    world_name: str
    drone_name: str
    trajectory_type: str
    world: IWorld
    drone: IDrone
    skip_logging:bool
    # Optional parameters for different trajectory types
    pp_trajectory_filename: Optional[str] = None

    def experiment_from_manip(
        self, world_file, trajectory_file, drone_file
    ) -> Tuple[int, np.ndarray, np.ndarray, np.ndarray, IWorld, IDrone]:
        #! setup world & drone
        world = WorldParser.Parse(world_file)
        drone = DroneParser.Parse(drone_file, world)
        #! get trajectory
        delta_timestamp, longitude, latitude, heading = TrajectoryParser.read_pbp(
            trajectory_file, ref=world.reference_point
        )
        return delta_timestamp, longitude, latitude, heading, world, drone

    def run(self):
        if self.trajectory_type in "pp":
            (
                delta_timestamp,
                longitude_array,
                latitude_array,
                heading_array,
                self.world,
                self.drone,
            ) = self.experiment_from_manip(
                world_file=self.world_name,
                trajectory_file=self.pp_trajectory_filename,
                drone_file=self.drone_name,
            )
        #! activate the clock
        clock = Clock()
        clock.set_conversion_factor(delta_timestamp)
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
            clock.increment_time()
            if(not self.skip_logging):
                drone_logger.log(self.drone)
        print("experiment >>>>> experiment ended saving in progress ...")
        drone_logger.wait_until_complete()

    def __init__(self, name):
        self.name = name
