import os

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ------------------------------------------------------------
    # MoveIt Config Builder
    # ------------------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder(
            "LeKiwi",
            package_name="lekiwi_moveit_pkg"
        )
        .to_moveit_configs()
    )

    # ------------------------------------------------------------
    # Robot State Publisher
    # ------------------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True}
        ],
    )

    # ------------------------------------------------------------
    # Move Group
    # ------------------------------------------------------------
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ],
    )

    # ------------------------------------------------------------
    # RViz
    # ------------------------------------------------------------
    rviz_config_file = os.path.join(
        FindPackageShare("lekiwi_moveit_pkg").find("lekiwi_moveit_pkg"),
        "config",
        "moveit.rviz"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        move_group,
        rviz,
    ])
