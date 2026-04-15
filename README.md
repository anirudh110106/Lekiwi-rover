# Lekiwi-rover
```

mobile-manipulator/
├── lekiwi_bringup/
│   ├── config/
│   │   └── controllers.yaml
│   ├── launch/
│   │   ├── launch.py
│   │   ├── sim.launch.py 
│   │   ├── (few controller test files)
│   ├── lekiwi_bringup/
│   │   └── motor_odom.py 

│
├── lekiwi_description/
│   ├── ros2_control/
│   │   ├── lekiwi_ros2_control.xacro (For Simulation)
│   │   └── lekiwi_ros2_control_hardware.xacro 
│   ├── URDF/
│   │   ├── LeKiwi.urdf
│   │   ├── lekiwi.urdf.xacro
│   │   ├── lekiwi_hardware.urdf.xacro
│   │   └── meshes/

│
├── lekiwi_moveit_config/
│   └── lekiwi_moveit_config/
│       ├── config/
│       └── launch/
│
├── nav2/
│   ├── nav2/
│   │   ├── launch/
│   │   │   ├── lidar_nav2.py (Lidar)
│   │   │   ├── nav2.py (Depth To Laser)
│   │   │   └── slam.py (Lidar)
│   │   ├── dwb_params.yaml - (Dynamic Window approach for Nav2)
│   │   ├── laser.py (Depth To Laser)
│   │   ├── my_nav2_params.yaml  - (Mppi approach for Nav2)
│   │   ├── my_new_map.pgm
│   │   ├── my_new_map.yaml
│   │   └── odom.py - (odometry for testing)
│   ├── package.xml
│   └── setup.py
│
├── st3215_hardware_interface/
│   ├── include/
│   ├── src/
│        └── st3215_system.cpp 
|  
│
└── rplidar_ros/ (ros2 package for lidar)
```
