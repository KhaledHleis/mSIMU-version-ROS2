import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

from sim_interfaces.msg import OdometryPath, SensorMeasurements
import json
import numpy as np

from sim_core.experiment import Experiment

from scipy.spatial.transform import Rotation


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

        # Optional realtime plot (off by default so the node is headless-safe).
        self.declare_parameter("enable_plot", False)
        self.enable_plot = bool(self.get_parameter("enable_plot").value)

        self.experiment = Experiment(config)
        self.get_logger().info(f"Experiment initialised: {self.experiment}")

        self.sub_drone_path = self.create_subscription(
            OdometryPath, "drone_path", self.drone_path_callback, 100
        )

        # Structured table (one message per received path segment) that carries
        # every sensor's magnetic field samples. This is what downstream
        # detection / localisation nodes consume.
        self.pub_sensor_data = self.create_publisher(
            SensorMeasurements, "sensor_data", 100
        )

        self.get_logger().info("MagSIMUNode has been started.")
        if self.enable_plot:
            self.simple_plotter_init()

    def drone_path_callback(self, msg):
        # self.get_logger().info(f"Received drone path with {len(msg.poses)} poses.")
        # debug calculation time
        start_time = self.get_clock().now()
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

        # measurements_array shape: (M sensors, N samples, 3), sensor_names: (M,)
        measurements_array, sensor_names = self.experiment.batch_measurements_and_updates(
            path, times, out_array=True
        )
        self.get_logger().info(
            f"calculation time: {(self.get_clock().now() - start_time).nanoseconds / 1e6:.2f} ms"
        )
        if self.enable_plot:
            self.simple_plotter_update(self.plotter, measurements_array, sensor_names)

        # ---- publish sensor data as a structured table --------------------
        # One SensorMeasurements message carries, per sensor, the N magnetic
        # field samples computed for the N poses in this path segment. The
        # header stamp mirrors the incoming path so downstream nodes can align
        # measurements with the drone poses that produced them.
        self._publish_measurements(msg.header, measurements_array, sensor_names, times)

        # self.csv_manager(self.experiment.batch_CSV_updates(path, times)) #option to save csv data

    def _publish_measurements(self, path_header, measurements_array, sensor_names, times):
        """Publish one SensorMeasurements message per sensor for this segment."""
        measurements_array = np.asarray(measurements_array)
        if measurements_array.ndim != 3:
            return
        n_samples = measurements_array.shape[1]

        for s_idx, name in enumerate(sensor_names):
            out = SensorMeasurements()
            out.header = path_header
            out.sensor_name = str(name)

            fields = []
            for k in range(n_samples):
                mf = MagneticField()
                # per-sample timestamp (ns -> ROS time)
                t_ns = int(times[k])
                mf.header.stamp.sec = t_ns // 1_000_000_000
                mf.header.stamp.nanosec = t_ns % 1_000_000_000
                mf.header.frame_id = str(name)
                bx, by, bz = np.asarray(measurements_array[s_idx, k]).reshape(3)
                mf.magnetic_field.x = float(bx)
                mf.magnetic_field.y = float(by)
                mf.magnetic_field.z = float(bz)
                fields.append(mf)

            out.magnetic_field = fields
            self.pub_sensor_data.publish(out)

    # region simple plotter
    def simple_plotter_init(self):
        """
        Creates a realtime matplotlib plotter (lazy import, opt-in only).
        """
        import matplotlib.pyplot as plt

        plt.ion()  # interactive mode ON
        self.plotter = {
            "initialized": False,
            "sensor_names": [],
        }
        return self.plotter

    def simple_plotter_update(self, plotter, measurements_array, sensor_names):
        """
        Appends new norm(data) values to the existing realtime plot.
        """
        import matplotlib.pyplot as plt

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
