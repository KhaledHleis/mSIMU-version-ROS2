# simulation_ros2 (mSIMU)

A ROS2 Python package for simulating magnetic field measurements onboard a
drone.  Given a georeferenced flight path, it computes the expected readings
of one or more magnetometers by summing a regional background field and the
contributions of user-defined magnetic sources (cables, dipoles).

---

## Overview

The simulator is built around three layers:

| Layer | Description |
|---|---|
| **sim_core** | Pure-Python simulation engine — no ROS dependency |
| **simulation_ros2** | ROS2 node that wires `sim_core` to topics |
| **config/** | YAML parameter file consumed by the launch system |

This separation means `sim_core` can be imported and tested independently of
ROS2, which makes unit-testing and offline batch processing straightforward.

---

## Architecture

```
simulation_ros2/
├── simulation_ros2/
│   └── magnetic_simulation_node.py   ← ROS2 node
│
├── sim_core/
│   ├── experiment.py                 ← top-level orchestrator
│   ├── interfaces/                   ← abstract base classes
│   │   ├── drone_interface.py        (IDrone)
│   │   ├── sensor_interface.py       (ISensor)
│   │   ├── target_interface.py       (ITarget)
│   │   └── world_interface.py        (IWorld)
│   ├── metaclasses/
│   │   ├── simu_class.py             (SIMU base — name + __str__)
│   │   ├── singleton.py              (Singleton metaclass)
│   │   └── string_convertable.py
│   ├── simu_objects/
│   │   ├── clock.py                  (Singleton clock)
│   │   ├── drone.py                  (Drone)
│   │   ├── sensor.py                 (Fluxgate, Scalar)
│   │   ├── target.py                 (Cable, Dipole)
│   │   └── world.py                  (World)
│   └── utils/
│       ├── utilities_converter.py    (LLD ↔ NED, body ↔ NED rotations)
│       └── utilities_initiliser.py   (build World + Drone from config dict)
│
├── config/Test.yaml
└── launch/mag_simu.launch.py
```

### Class diagram

```
ITarget  ◄──── Target ◄──── Cable
                      ◄──── Dipole

ISensor  ◄──── Sensor ◄──── Fluxgate   (3-component vector magnetometer)
                      ◄──── Scalar      (total-field magnetometer)

IWorld   ◄──── World       (holds N targets + regional field)
IDrone   ◄──── Drone       (holds M sensors + reference to World)

Experiment  ──owns──►  World
            ──owns──►  Drone
            ──uses──►  Clock  (Singleton)
```

All concrete classes inherit from `SIMU` (which provides `name` and
`__str__`) and their respective interface (which defines the contract).

### Singleton Clock

`Clock` uses the `Singleton` metaclass, meaning every call to `Clock()`
returns the same instance regardless of where it is called.  It stores the
current simulation timestamp and the last `delta_t`, both in nanoseconds.

### ROS2 interfaces

| Direction | Topic | Message type | Description |
|---|---|---|---|
| Subscribed | `/drone_path` | `sim_interfaces/OdometryPath` | Sequence of drone poses |
| Published | `/sensor_data` | `sensor_msgs/MagneticField` | Measurement of sensor 0 |

The node also opens a **real-time matplotlib window** that plots the
zero-centred norm of all sensor measurements as the path is processed.

An optional **CSV export** is available (disabled by default — see source
comments in `magnetic_simulation_node.py`).

---

## Dependencies

### System

```bash
sudo apt update
sudo apt install python3-pip \
                 ros-humble-rclpy \
                 ros-humble-sensor-msgs \
                 ros-humble-geometry-msgs
```

### Python

```bash
pip install numpy scipy pyproj matplotlib
```

| Package | Role |
|---|---|
| `numpy` | Matrix / vector math |
| `scipy` | Quaternion → Euler conversion (`Rotation.from_quat`) |
| `pyproj` | LLD ↔ NED coordinate conversion (Transverse Mercator) |
| `matplotlib` | Real-time measurement plotter |

### ROS2 custom messages

The package depends on `sim_interfaces` (provides `OdometryPath`).
Make sure it is built and sourced in the same workspace.

---

## Building

```bash
cd ~/ros2_ws_simu
source /opt/ros/humble/setup.bash

colcon build --packages-select simulation_ros2
source install/setup.bash
```

---

## Configuration

Parameters are loaded from a YAML file via the launch system.
All geographic coordinates use **decimal degrees** for longitude/latitude
and **metres** for depth (positive = downward).

```yaml
# config/Test.yaml
mag_simu_node:
  ros__parameters:

    experiment: '{"name": "bb_S90"}'

    world: '{
      "name": "WT1",
      "reference_longitude": -4.503754158369759,
      "reference_latitude":  48.49209220724089,
      "reference_depth":     0,
      "simulation_radius":   50,
      "regional_magnetic_field": [21271, -100, 43023]
    }'
    # regional_magnetic_field: [North, East, Down] in nanoTesla

    cables: '[{
      "name":               "Cable_south",
      "starting_longitude": -4.503827049110521,
      "starting_latitude":  48.492313209033945,
      "starting_depth":     1.5,
      "ending_longitude":   -4.50434496199812,
      "ending_latitude":    48.49227909392259,
      "ending_depth":       1.0,
      "current":            6,
      "current_frequency":  0
    }]'

    drone: '{"name": "drone_cyclope"}'

    sensors: '[{
      "name":              "sensor_UNO",
      "type":              "Fluxgate",
      "relative_position": [0, 0, 0]
    }]'
```

### Sensor types

| Type | Output shape | Description |
|---|---|---|
| `Fluxgate` | `(1, 3)` — `[Bx, By, Bz]` nT in body frame | 3-component vector magnetometer |
| `Scalar` | scalar — `‖B‖` nT | Total-field magnetometer |

### Adding more targets

Add extra objects to the JSON arrays in the YAML.

**Additional cable:**
```yaml
cables: '[
  {"name": "Cable_south", ...},
  {"name": "Cable_north",
   "starting_longitude": -4.5035, "starting_latitude": 48.4930,
   "starting_depth": 1.0,
   "ending_longitude": -4.5045, "ending_latitude": 48.4929,
   "ending_depth": 1.0,
   "current": 10, "current_frequency": 0}
]'
```

**Dipole source:**
```yaml
dipoles: '[{
  "name":             "dipole_A",
  "center_longitude": -4.5040,
  "center_latitude":  48.4921,
  "center_depth":     2.0,
  "dipole_moment":    [0, 0, 1000000]
}]'
```

### Adding more sensors

`relative_position` is expressed in the **body frame** `[x, y, z]` in metres
(x = forward, y = right, z = down).

```yaml
sensors: '[
  {"name": "fwd",   "type": "Fluxgate", "relative_position": [ 0.5, 0, 0]},
  {"name": "rear",  "type": "Fluxgate", "relative_position": [-0.5, 0, 0]},
  {"name": "total", "type": "Scalar",   "relative_position": [0,    0, 0]}
]'
```

---

## Running

```bash
ros2 launch simulation_ros2 mag_simu.launch.py
```

To use a custom config file, edit the `Configuration_file` variable at the
top of `launch/mag_simu.launch.py`, or pass a full path:

```bash
ros2 launch simulation_ros2 mag_simu.launch.py \
  config:=<absolute_path_to_your_config.yaml>
```

---

## Real-time Plotter

When the node starts it opens a **matplotlib** window automatically.
It plots the zero-centred norm `‖B‖ − mean(‖B‖)` of all sensor measurements
in real time, updating every time a new `drone_path` message arrives.

No extra installation is needed beyond `matplotlib`.

---

## Optional CSV Export

Every processed path can be saved to a CSV file.  The feature is disabled
by default.  To enable it, uncomment this line in `drone_path_callback`:

```python
# self.csv_manager(self.experiment.batch_CSV_updates(path, times))
```

The CSV columns are: `timestamp, longitude, latitude, depth, heading,
magx, magy, magz, mag`.

---

## Using `sim_core` Without ROS2

The simulation engine is fully independent of ROS2 and can be used in any
Python script:

```python
import json
import numpy as np
from sim_core.experiment import Experiment

config = {
    "experiment": {"name": "offline_test"},
    "world": {
        "name": "WT1",
        "reference_longitude": -4.5038,
        "reference_latitude":  48.4921,
        "reference_depth":     0,
        "simulation_radius":   50,
        "regional_magnetic_field": [21271, -100, 43023]
    },
    "cables": [{
        "name": "Cable_south",
        "starting_longitude": -4.5038, "starting_latitude": 48.4923,
        "starting_depth": 1.5,
        "ending_longitude": -4.5043,   "ending_latitude": 48.4923,
        "ending_depth": 1.0,
        "current": 6, "current_frequency": 0
    }],
    "drone":   {"name": "drone_test"},
    "sensors": [{"name": "s0", "type": "Fluxgate",
                 "relative_position": [0, 0, 0]}]
}

exp = Experiment(config)

# Single update
exp.update([−4.5038, 48.4921, 1.0, 0.0, 0.0, 0.0], timestamp=0)
print(exp.get_measurements())

# Batch update
path = np.array([
    [-4.5038, 48.4921, 1.0, 0, 0, 0],
    [-4.5039, 48.4921, 1.0, 0, 0, 0],
])
times = [0, 100_000_000]  # nanoseconds
results, sensor_names = exp.batch_measurements_and_updates(
    path, times, out_array=True
)
# results shape: (n_sensors, n_steps, 3)
```

---

## Coordinate Frames

| Frame | Axes | Used for |
|---|---|---|
| **LLD** | Longitude (°), Latitude (°), Depth (m ↓) | Config file, incoming poses |
| **NED** | North (m), East (m), Down (m) | Internal simulation, field vectors |
| **Body** | x = forward, y = right, z = down | Sensor offsets, Fluxgate output |

Conversion chain for each pose:

```
Pose (LLD)  ──lld_to_ned()──►  Drone NED  ──body_to_ned()──►  Sensor NED
                                                                    │
                                               World.calculate_entire_field_at_position()
                                                                    │
                                Fluxgate  ──ned_to_body()──►  Body-frame [Bx, By, Bz]
                                Scalar    ──np.linalg.norm()──►  ‖B‖
```

---

## License

MIT