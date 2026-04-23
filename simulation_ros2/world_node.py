# ros imports
import rclpy
from rclpy.node import Node


# local ros imports
from sim_interfaces.srv import DebugWorldName, CalculateField

# python packages
import numpy as np

# local imports
from sim_core.simu_objects.world import World

class WorldNode(Node):
    """ node to interact with the world, e.g. to spawn objects, get information about the world, etc. """
    world: World
    
    def __init__(self):
        super().__init__('world_node')
        # for testing purposes, we will initialize a world here
        self.world = World("world",[])
        self.srv_world_name = self.create_service(DebugWorldName, 'world_name', self.world_name_callback)
        self.srv_calculate_field = self.create_service(CalculateField, 'calculate_field', self.calculate_field_callback)
        self.get_logger().info('WorldNode has been started.')
    
    def calculate_field_callback(self, request, response):
        position = np.array([request.pos_x, request.pos_y, request.pos_z])
        field = self.world.calculate_entire_field_at_position(position)
        
        response.field_x = field[0,0]
        response.field_y = field[0,1]
        response.field_z = field[0,2]
        return response

    def world_name_callback(self, request, response):
        response.message = self.world.name
        response.success = True
        self.get_logger().info(f"Received request for world name. Responding with: {response.message}")
        return response

def main(args=None):
    rclpy.init(args=args)
    world_node = WorldNode()
    rclpy.spin(world_node)
    rclpy.shutdown()