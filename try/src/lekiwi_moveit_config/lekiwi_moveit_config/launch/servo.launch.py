from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():

    # 1. Build the MoveIt config. 
    # Passing "lekiwi" tells it to look in the "lekiwi_moveit_config" package
    # This automatically loads the URDF, SRDF, and kinematics files.
    moveit_config = MoveItConfigsBuilder("lekiwi").to_moveit_configs()

    # 2. Get the specific servo parameters
    pkg = get_package_share_directory("lekiwi_moveit_config")
    servo_params = os.path.join(pkg, "config", "servo.yaml")

    # 3. Launch the servo node with BOTH the servo params and the robot description
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.to_dict(), # Inject robot_description & robot_description_semantic
        ],
    )

    return LaunchDescription([
        servo_node
    ])