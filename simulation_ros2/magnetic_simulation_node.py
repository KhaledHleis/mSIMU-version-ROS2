import rclpy
from rclpy.node import Node
import sim_interfaces.msg
import json

from sim_core.experiment import Experiment


class MagSIMUNode(Node):
    experiment: Experiment

    def __init__(self):
        super().__init__('mag_simu_node')

        # declare parameters matching the yaml keys
        self.declare_parameter('experiment', '')
        self.declare_parameter('world',      '')
        self.declare_parameter('cables',     '')
        self.declare_parameter('drone',      '')
        self.declare_parameter('sensors',    '')

        # build config dict by parsing each JSON string from the yaml
        config = {
            "experiment": json.loads(self.get_parameter('experiment').value),
            "world":      json.loads(self.get_parameter('world').value),
            "cables":     json.loads(self.get_parameter('cables').value),
            "drone":      json.loads(self.get_parameter('drone').value),
            "sensors":    json.loads(self.get_parameter('sensors').value),
        }

        self.experiment = Experiment(config)
        self.get_logger().info(f'Experiment initialised: {self.experiment}')

        self.sub_drone_path = self.create_subscription(
            sim_interfaces.msg.OdometryPath,
            'drone_path',
            self.drone_path_callback,
            100
        )

        self.get_logger().info('MagSIMUNode has been started.')

    def drone_path_callback(self, msg):
        # transform path from satellite coordinates to world coordinates
        # get recordings and publish to output topic
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MagSIMUNode()
    rclpy.spin(node)
    rclpy.shutdown()