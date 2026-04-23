# ros imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

# python packages
import numpy as np

# local imports
from simulation_logic.simu_objects.world import World

class WorldNode(Node):
    """ node to interact with the world, e.g. to spawn objects, get information about the world, etc. """
    world: World
    
    def __init__(self):
        super().__init__('world_node')
        self.srv_calculate_field = self.create_service(Vector3, 'calculate_field', self.calculate_field_callback)
        self.get_logger().info('WorldNode has been started.')
    
    def calculate_field_callback(self, request, response):
        position = np.array([request.x, request.y, request.z])
        field = self.world.calculate_entire_field_at_position(position)
        response.x = field[0]
        response.y = field[1]
        response.z = field[2]
        return response