# LeKiwi Rover

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/index.html)
[![ros2_control](https://img.shields.io/badge/ros2__control-hardware%20interface-orange.svg)](https://control.ros.org/)
[![MoveIt2](https://img.shields.io/badge/MoveIt2-motion%20planning-9cf.svg)](https://moveit.ros.org/)
[![Platform](https://img.shields.io/badge/Platform-Three--Wheel%20Holonomic-lightgrey.svg)](#)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](#license)

A ROS2-based control stack for the **LeKiwi** three-wheel omnidirectional rover, built entirely on open-source STL and URDF files. The system integrates **ST3215** servo motors through a custom `ros2_control` hardware interface, mapping individual motors to joints with dedicated position and velocity control modes, and coordinates rover motion through three distinct controllers — drive wheels, robotic arm, and end-effector gripper. Arm motion planning is fully supported via a dedicated **MoveIt2** package.

> **Navigation (Nav2):** SLAM and autonomous navigation for this rover live in a separate repo — see [anirudh110106/Nav2](https://github.com/anirudh110106/Nav2).

---

## Overview

LeKiwi combines:
- A **holonomic three-wheel drive base** for omnidirectional movement
- A **robotic arm + gripper** built from ST3215 servos
- A **custom hardware interface** bridging physical servos to `ros2_control`
- **MoveIt2** integration for arm motion planning

---

## Package Structure

```

mobile-manipulator/
├── lekiwi_bringup/
│   ├── config/
│   │   └── controllers.yaml
│   ├── launch/
│   │   ├── launch.py
│   │   ├── sim.launch.py
│   │   ├── (Controllers test files)
│   ├── lekiwi_bringup/
│   │   └── motor_odom.py 
├── lekiwi_description/
│   ├── ros2_control/
│   │   ├── lekiwi_ros2_control.xacro (For Simulation)
│   │   └── lekiwi_ros2_control_hardware.xacro
│   ├── URDF/
│   │   ├── meshes/
│   │   ├── LeKiwi.urdf
│   │   ├── lekiwi.urdf.xacro
│   │   └── lekiwi_hardware.urdf.xacro
├── lekiwi_moveit_config/
│   └── lekiwi_moveit_config/
│       ├── config/
│       ├── launch/
├── nav2/
│   ├── nav2/
│   │   ├── launch/
│   │   │   ├── depth_laser.py (Using RealSence Depth Camera)
│   │   │   ├── lidar_nav2.py
│   │   │   ├── nav2.py (Nav2 Using RealSence)
│   │   │   └── slam.py (For lidar)
│   │   ├── map/ 
│   │   ├── params/
│   │   ├── temp/
│   │   └── odom.py 
├── rplidar_ros/ (ros2 package for lidar)
├── st3215_hardware_interface/
│   ├── include/
│   ├── src/
│   │   └── st3215_system.cpp 
└── README.md


```

### `lekiwi_bringup`
Core launch and runtime package.
- `launch/launch.py` — bring up the real rover
- `launch/sim.launch.py` — bring up in simulation
- `launch/test_wheels.py`, `launch/test_gripper.py`, `launch/test_joint_traj.py` — isolated hardware test launches
- `lekiwi_bringup/motor_odom.py` — odometry computation from motor feedback
- `config/controllers.yaml` — controller definitions (drive, arm, gripper)

### `lekiwi_description`
Robot model package.
- `URDF/LeKiwi.urdf`, `URDF/lekiwi.urdf.xacro`, `URDF/lekiwi_hardware.urdf.xacro` — robot description
- `URDF/meshes/` — STL meshes for chassis, omni wheels, arm links, camera mounts, etc.
- `ros2_control/lekiwi_ros2_control.xacro`, `lekiwi_ros2_control_hardware.xacro` — ros2_control tag definitions
- `launch/sim.launch.py` — simulation-only robot state publishing

### `lekiwi_moveit_config`
MoveIt2 configuration for the arm, generated via MoveIt Setup Assistant.
- `config/LeKiwi.srdf` — semantic robot description (planning group `arm`, EEF link `Wrist_Roll_08c-v1`)
- `config/kinematics.yaml`, `config/joint_limits.yaml`, `config/ompl_planning.yaml`, `config/pilz_cartesian_limits.yaml`
- `config/moveit_controllers.yaml`, `config/ros2_controllers.yaml` — controller bridging
- `config/servo.yaml` — MoveIt Servo (real-time Cartesian jogging) config
- `launch/demo.launch.py` — full MoveIt demo with RViz
- `launch/move_group.launch.py`, `launch/servo.launch.py`, `launch/moveit_rviz.launch.py`, `launch/rsp.launch.py`, `launch/spawn_controllers.launch.py`, `launch/static_virtual_joint_tfs.launch.py`

### `st3215_hardware_interface`
Custom C++ `ros2_control` hardware interface for ST3215 serial-bus servos.
- `src/st3215_system.cpp` — hardware interface implementation (read/write loop, position/velocity modes)
- `include/st_sc_servo_control_lib/` — servo control library (SCS, SCSCL, SMS_STS protocol handlers)
- `st3215_hardware_interface.xml` — plugin export for `pluginlib`

---

## Prerequisites

- Ubuntu 24.04 + ROS2 Jazzy
- `ros2_control`, `ros2_controllers`
- `MoveIt2`
- Serial access to ST3215 servo bus (USB-to-serial adapter, correct udev permissions)

```bash
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-moveit \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher
```

---

## Build

```bash
cd ~/your_ws/src
git clone https://github.com/anirudh110106/Lekiwi-rover.git
cd ~/your_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## Usage

### Bring up the rover

**Real hardware:**
```bash
ros2 launch lekiwi_bringup launch.py
```

**Simulation:**
```bash
ros2 launch lekiwi_bringup sim.launch.py
```

<img width="12447" height="3590" alt="image" src="https://github.com/user-attachments/assets/33f95743-7403-45ca-b0b4-fb5950859c3f" />


### Hardware tests

```bash
ros2 launch lekiwi_bringup test_wheels.py
ros2 launch lekiwi_bringup test_gripper.py
ros2 launch lekiwi_bringup test_joint_traj.py
```

### Arm motion planning (MoveIt2)

```bash
ros2 launch lekiwi_moveit_config demo.launch.py
```

For real-time Cartesian jogging:
```bash
ros2 launch lekiwi_moveit_config servo.launch.py
```

---

## Navigation

Autonomous navigation and SLAM for this rover are maintained in a separate repository:
👉 **[anirudh110106/Nav2](https://github.com/anirudh110106/Nav2)**

---

## License
This project is intended for educational and robotics research purposes.


MIT — see individual package `LICENSE` files for details.
