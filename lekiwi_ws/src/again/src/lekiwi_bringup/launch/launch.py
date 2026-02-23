import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    description_path = get_package_share_directory('lekiwi_description')
    bringup_path = get_package_share_directory('lekiwi_bringup')

    # ---- Process Hardware Xacro ----
    xacro_file = os.path.join(
        description_path,
        'URDF',
        'lekiwi_hardware.urdf.xacro'
    )

    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    robot_description = {'robot_description': robot_desc}

    # ---- Controller Config File ----
    controller_config = os.path.join(
        bringup_path,
        'config',
        'controllers.yaml'
    )

    # ---- ros2_control_node (THIS STARTS controller_manager) ----
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen'
    )

    # ---- Robot State Publisher ----
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # ---- Controller Spawners ----
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_position_controller'],
        output='screen'
    )

    base_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['base_velocity_controller'],
        output='screen'
    )

    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        control_node,
        joint_state_spawner,
        arm_spawner,
        base_spawner,
        gripper_spawner,
    ])