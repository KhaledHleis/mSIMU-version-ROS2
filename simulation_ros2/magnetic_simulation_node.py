# ros imports
import rclpy
from rclpy.node import Node


# local ros imports
import sim_interfaces.msg

# python packages
import numpy as np

# local imports
from sim_core.experiment import Experiment

class MagSIMUNode(Node):
    """ node to interact with the world, e.g. to spawn objects, get information about the world, etc. """
    experiment:Experiment
    def __init__(self):
        super().__init__('mag_simu_node')
        # initialise experiment and python simulation
        fake_object = { # this will be replaced with file parsing
            "experiment":{"name":"bb_S90"},
            "world":{
                        "name": "WT1",
                        "reference_longitude": -4.503754158369759,
                        "reference_latitude": 48.49209220724089,
                        "reference_depth": 0,
                        "simulation_radius": 50,
                        "regional_magnetic_field": [21271, -100, 43023],
            },
            "cables": [
                        {
                        "name": "Cable_south",
                        "starting_longitude": -4.503827049110521,
                        "starting_latitude": 48.492313209033945,
                        "starting_depth": 1.5,
                        "current": 6,
                        "current_frequency": 0,
                        "ending_longitude": -4.50434496199812,
                        "ending_latitude": 48.49227909392259,
                        "ending_depth": 1
                        }
            ],
            "drone":{
                        "name": "drone_cyclope",
                        
            },
            "sensors": [
                            { "name": "sensor_UNO", "relative_position": [0, 0, 0], "type": "Fluxgate" }
            ]
        }
        self.experiment = Experiment(fake_object)
        print(self.experiment)
        # here subscribe to the topic (drone path)
        self.sub_drone_path = self.create_subscription(sim_interfaces.msg.OdometryPath, 'drone_path', self.drone_path_callback, 100)
        self.get_logger().info('MagSIMUNode has been started.')
    
    def drone_path_callback(self, msg):
        # transform the path from satellite coordinates to the world coordinates and update the drone position in the world
        # get recordings and send them in output topic
        pass
    

def main(args=None):
    rclpy.init(args=args)
    world_node = MagSIMUNode()
    rclpy.spin(world_node)
    rclpy.shutdown()