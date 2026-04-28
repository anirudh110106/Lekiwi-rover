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
