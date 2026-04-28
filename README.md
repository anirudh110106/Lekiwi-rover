### Lekiwi Rover

ROS 2 System Architecture & Control
The ROS2 ecosystem for the Lekiwi rover is built by utilizing the open-source STL's and  URDF files. To bridge the physical hardware with the software stack, the system integrates ST3215 motor packages managed by a custom ros2_control interface. This maps individual motors to specific joints and establishes dedicated position and velocity control modes. The rover's movement is orchestrated through three distinct controllers explicitly designed to manage the drive wheels, the robotic arm, and the end-effector gripper.Arm motion planning fully supported through a dedicated MoveIt package.


# Setup 

Create the workspace :-
```
cd ~
mkdir Rover
mkdir Rover/src
cd ~Rover/src
```
Clone the repository :-
```
git clone https://github.com/KMTI-ROBOPARADIGM/mobile-manipulator/tree/main
```
Build the Package :-
```
cd ~Rover
colcon build
source install/setup.bash
```


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
