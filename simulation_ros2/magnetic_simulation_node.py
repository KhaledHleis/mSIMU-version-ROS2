import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

from sim_interfaces.msg import OdometryPath
import json
import numpy as np

from sim_core.experiment import Experiment

from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt


class MagSIMUNode(Node):
    experiment: Experiment
    _first_csv_save: bool = True
    def __init__(self):
        super().__init__("mag_simu_node")

        # declare parameters matching the yaml keys
        self.declare_parameter("experiment", "")
        self.declare_parameter("world", "")
        self.declare_parameter("cables", "")
        self.declare_parameter("drone", "")
        self.declare_parameter("sensors", "")

        # build config dict by parsing each JSON string from the yaml
        config = {
            "experiment": json.loads(self.get_parameter("experiment").value),
            "world": json.loads(self.get_parameter("world").value),
            "cables": json.loads(self.get_parameter("cables").value),
            "drone": json.loads(self.get_parameter("drone").value),
            "sensors": json.loads(self.get_parameter("sensors").value),
        }

        self.experiment = Experiment(config)
        self.get_logger().info(f"Experiment initialised: {self.experiment}")

        self.sub_drone_path = self.create_subscription(
            OdometryPath, "drone_path", self.drone_path_callback, 100
        )

        self.pub_sensor_data = self.create_publisher(MagneticField, "sensor_data", 100)

        self.get_logger().info("MagSIMUNode has been started.")
        self.simple_plotter_init()

    def drone_path_callback(self, msg):
        self.get_logger().info(f"Received drone path with {len(msg.poses)} poses.")

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
            roll, pitch, yaw = r.as_euler("xyz", degrees=False)

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
            times.append(
                odom.header.stamp.sec * 1e9 + odom.header.stamp.nanosec
            )  # convert to ns

        path = np.stack(
            (x_poses, y_poses, z_poses, roll_rots, pitch_rots, yaw_rots), axis=1
        )

        measurements_array,sensor_names = self.experiment.batch_measurements_and_updates(path, times, out_array=True)
        self.simple_plotter_update(self.plotter,measurements_array, sensor_names)


        # self.csv_manager(self.experiment.batch_CSV_updates(path, times)) #option to save csv data

        # WORK put the output in the correct msg format
        # publish sensor data as table
        # self.get_logger().info(str(measurements_array))

    # region simple plotter
    def simple_plotter_init(self):
        """
        Creates a realtime matplotlib plotter.
        One subplot per sensor column.
        """
        plt.ion()  # interactive mode ON

        fig, axes = plt.subplots(nrows=1, ncols=1, figsize=(10, 6), squeeze=False)

        plotter = {
            "fig": fig,
            "axes": axes.flatten(),
            "lines": [],
            "initialized": False,
            "sensor_names": [],
        }

        self.plotter = plotter
        return plotter

    def simple_plotter_update(self, plotter, measurements_array, sensor_names):
        """
        Appends new norm(data) values to the existing realtime plot.
        """

        if measurements_array is None or len(measurements_array) == 0:
            return

        # Compute norm over last axis
        data = np.linalg.norm(np.asarray(measurements_array), axis=-1).reshape(-1)

        # ---------- First-time setup ----------
        if not plotter.get("initialized", False):

            fig, ax = plt.subplots(figsize=(10, 4))
            (line,) = ax.plot([], [], lw=2)

            ax.set_title("Realtime Measurements Norm")
            ax.set_xlabel("Sample Index")
            ax.set_ylabel("Norm")
            ax.grid(True)

            plotter["fig"] = fig
            plotter["ax"] = ax
            plotter["line"] = line
            plotter["xdata"] = []
            plotter["ydata"] = []
            plotter["initialized"] = True

        # ---------- Append new data ----------
        start_idx = len(plotter["ydata"])

        for i, val in enumerate(data):
            plotter["xdata"].append(start_idx + i)
            plotter["ydata"].append(val)

        x = np.asarray(plotter["xdata"])
        y = np.asarray(plotter["ydata"])
        y = y - np.mean(y)  # zero-center for better visualization

        # ---------- Update line ----------
        plotter["line"].set_data(x, y)

        plotter["ax"].set_xlim(0, max(10, len(x)))

        y_min = np.min(y)
        y_max = np.max(y)
        pad = 1.0 if np.isclose(y_min, y_max) else 0.1 * (y_max - y_min)

        plotter["ax"].set_ylim(y_min - pad, y_max + pad)

        # ---------- Redraw ----------
        plotter["fig"].canvas.draw()
        plotter["fig"].canvas.flush_events()
        plt.pause(0.001)

    # endregion

    # region CSV saver
    def csv_manager(self, csv_data, filename="output.csv"):
        """
        Saves the provided CSV data (list of lists) to a file.
        """
        import csv

        if self._first_csv_save:
            # If the file doesn't exist, write headers
            full_filename = ""
            with open(filename, mode="w", newline="") as file:
                writer = csv.writer(file)
                writer.writerows(csv_data)
                full_filename = file.name
            self._first_csv_save = False
            self.get_logger().info(f"CSV data saved to {full_filename}")
        else:
            with open(filename, mode="a", newline="") as file:
                writer = csv.writer(file)
                writer.writerows(csv_data[1:])  # Skip header for subsequent saves

    # endregion


def main(args=None):
    rclpy.init(args=args)
    node = MagSIMUNode()
    rclpy.spin(node)
    rclpy.shutdown()
