# FAST_LIO_NAV2

This repository provides an experimental mapless, short-distance autonomous navigation system. It seamlessly integrates real-time LiDAR odometry and point cloud generation from FAST_LIO with the ROS 2 Nav2 stack.

Developed by the Adaptive Learning Systems Control Laboratory (Okayama University).

## 🖥️ System Requirements
* **OS:** Ubuntu 22.04
* **ROS 2:** Humble Hawksbill
* **Hardware (Compute):** Intel NUC (Core i7) or NVIDIA Jetson Orin Nano
* **Hardware (Sensor):** Livox Mid-360 (Assumed to be mounted horizontally, facing upwards)

## 📦 Installation & Build
This repository uses a `.repos` file to manage external dependencies cleanly.

```bash

# 1. Clone this repository
git clone https://github.com/nomoto39/FAST_LIO_NAV2.git

# 2. Import dependencies (livox_ros_driver2)
vcs import < FAST_LIO_NAV2/dependencies.repos

# 3. Build the workspace
cd ~/FAST_LIO_NAV2
colcon build --symlink-install
source install/setup.bash

```

## 🚀 Usage

To launch the entire autonomous navigation system (Sensor, SLAM, Nav2, and Robot Controller), run:
```bash

ros2 launch robot_controller auto_local.launch.py

```

## ⚙️ Customization for Your Robot

When applying this system to your mobile robot, you MUST adjust the following parameters to match your hardware dimensions.
### 1. Static TF (Coordinate Transformations)

The system uses the following TF tree: map -> odom -> body -> base_link -> livox_frame.

* FAST_LIO publishes the dynamic TF from odom -> body.

* You need to adjust the static TFs in your launch file (auto_local.launch.py).

Example Setup (Modify x, y, z according to your LiDAR mounting position):
The following example assumes the Livox sensor is mounted 0.17m forward (x) and 0.63m upward (z) from the center of the robot's base (ground level).
```python

# 1. map -> odom (Offsets the starting position)
map_to_odom_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='map_to_odom',
    arguments=['--x', '0.17', '--y', '0.0', '--z', '0.63', ... , '--frame-id', 'map', '--child-frame-id', 'odom']
)

# 2. body -> base_link (Transform from LiDAR to Robot base)
rotbody_to_baselink_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='body_to_base_link',
    arguments=['--x', '-0.17', '--y', '0.0', '--z', '-0.63', ... , '--frame-id', 'body', '--child-frame-id', 'base_link']
)

# 3. base_link -> livox_frame (Transform from Robot base to LiDAR)
baselink_to_livox_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_tf_base_link_livox',
    arguments=['--x', '0.17', '--y', '0.0', '--z', '0.63', ... , '--frame-id', 'base_link', '--child-frame-id', 'livox_frame']
)

```

### 2. Nav2 Parameters (robot_radius)

Open your Nav2 configuration YAML file located at src/robot_controller/config/nav2_params.yaml and update the robot_radius (or footprint) parameter to match the physical size of your robot.

You must change this value in both the local_costmap and global_costmap sections to ensure safe obstacle avoidance.

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.3 # Change this to your robot's radius in meters

global_costmap:
  global_costmap:
    ros__parameters:
      robot_radius: 0.3 # Change this to your robot's radius in meters

```

## 🙏 Acknowledgments

This system is built upon the incredible work of the open-source community. We express our deepest gratitude to the original authors:

    FAST-LIO by hku-mars (Original SLAM algorithm)

    FAST_LIO_ROS2 by Taeyoung96 (ROS 2 porting)

    ros2serial_arduino by tomoswifty (Arduino serial communication base)