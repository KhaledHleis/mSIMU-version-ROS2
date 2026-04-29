import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

from sim_interfaces.msg import OdometryPath
import json
import numpy as np

from sim_core.experiment import Experiment

from scipy.spatial.transform import Rotation

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
            OdometryPath,
            'drone_path',
            self.drone_path_callback,
            100
        )
        
        self.pub_sensor_data = self.create_publisher(
            MagneticField,
            'sensor_data',
            100
        )

        self.get_logger().info('MagSIMUNode has been started.')


    def drone_path_callback(self, msg):
        self.get_logger().info(f'Received drone path with {len(msg.poses)} poses.')
        
        x_poses = []
        y_poses = []
        z_poses = []
        roll_rots = []
        pitch_rots = []
        yaw_rots = []
        times = []
        for i, odom in enumerate(msg.poses):
            pos = odom.pose.pose.position
            ori = odom.pose.pose.orientation

            r = Rotation.from_quat([ori.x, ori.y, ori.z, ori.w])
            roll, pitch, yaw = r.as_euler('xyz', degrees=False)

            # self.get_logger().info(
            #     f'[{i}] lon={pos.x:.6f}  lat={pos.y:.6f}  depth={pos.z:.6f} | '
            #     f'roll={roll:.4f}rad  pitch={pitch:.4f}rad  yaw={yaw:.4f}rad'
            # )
            
            x_poses.append(pos.x)
            y_poses.append(pos.y)
            z_poses.append(pos.z)
            roll_rots.append(roll)
            pitch_rots.append(pitch)
            yaw_rots.append(yaw)
            times.append(odom.header.stamp.sec * 1e9 + odom.header.stamp.nanosec ) # convert to ns
            
        path = np.stack((x_poses, y_poses, z_poses, roll_rots, pitch_rots, yaw_rots),axis=1)
        
        measurements_array = self.experiment.batch_measurements_and_updates(path, times)
        
        # publish sensor data as table
        self.get_logger().info(str(measurements_array))
        # WORK put the output in the correct msg format


def main(args=None):
    rclpy.init(args=args)
    node = MagSIMUNode()
    rclpy.spin(node)
    rclpy.shutdown()