# ros imports
import rclpy
from rclpy.node import Node


# local ros imports
import sim_interfaces.srv

# python packages
import numpy as np

# local imports
from sim_core.simu_objects.world import World

class MagSIMUNode(Node):
    """ node to interact with the world, e.g. to spawn objects, get information about the world, etc. """
    world: World
    
    def __init__(self):
        super().__init__('mag_simu_node')
        # for testing purposes, we will initialize a world here
        self.world = World("world",[])
        # here create a service that calculates the field recorded at a position (request: position, response: field,Clock_tick)
        # here create a topic that publiches the clock tick of the world, so that other nodes can subscribe to it and know when to update their state
        self.get_logger().info('MagSIMUNode has been started.')
    
    def calculate_field_callback(self, request, response):
        position = np.array([request.pos_x, request.pos_y, request.pos_z])
        field = self.world.calculate_entire_field_at_position(position)
        
        response.field_x = field[0,0]
        response.field_y = field[0,1]
        response.field_z = field[0,2]
        return response

def main(args=None):
    rclpy.init(args=args)
    world_node = MagSIMUNode()
    rclpy.spin(world_node)
    rclpy.shutdown()