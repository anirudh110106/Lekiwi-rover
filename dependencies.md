# System / ROS2 Dependencies

These are not pip packages — install via apt or rosdep. Use `requirements.txt` only for the Python packages layered on top.

## 🧱 Core

| Dependency | Notes |
|---|---|
| Ubuntu 24.04 (Noble) | Required base OS for ROS2 Jazzy |
| ROS2 Jazzy Jalisco | `sudo apt install ros-jazzy-desktop` |
| colcon | Build tool — `sudo apt install python3-colcon-common-extensions` |
| rosdep | `sudo apt install python3-rosdep` |
| xacro | `sudo apt install ros-jazzy-xacro` |
| robot_state_publisher | `sudo apt install ros-jazzy-robot-state-publisher` |

## ⚙️ ros2_control

| Package | apt package |
|---|---|
| ros2_control | `ros-jazzy-ros2-control` |
| ros2_controllers | `ros-jazzy-ros2-controllers` |
| controller_manager | included with `ros2_control` |
| hardware_interface | included with `ros2_control` (needed to build `st3215_hardware_interface`) |

## 🦾 MoveIt2

| Package | apt package |
|---|---|
| MoveIt2 | `ros-jazzy-moveit` |
| MoveIt Setup Assistant | `ros-jazzy-moveit-setup-assistant` |
| MoveIt Servo | `ros-jazzy-moveit-servo` |
| Warehouse DB (optional) | `ros-jazzy-warehouse-ros-mongo` |

## 🔌 Hardware

| Dependency | Notes |
|---|---|
| ST3215 serial-bus servos | USB-to-serial (TTL) adapter |
| udev rules | grant non-root read/write access to the serial device (e.g. `/dev/ttyUSB0`) |

## 📥 Install (quick reference)

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-moveit \
  ros-jazzy-moveit-setup-assistant \
  ros-jazzy-moveit-servo \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  python3-colcon-common-extensions \
  python3-rosdep

# Resolve any remaining deps declared in package.xml
cd ~/your_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 🐍 Python (pip)

See `requirements.txt`. Install after sourcing your ROS2 environment:

```bash
source /opt/ros/jazzy/setup.bash
pip install -r requirements.txt
```
