import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory("arp_assignment"),
        "rviz",
        "bvi_demo.rviz"
    )

    start_delay = LaunchConfiguration("start_delay")
    step_period = LaunchConfiguration("step_period")

    return LaunchDescription([

        DeclareLaunchArgument(
            "start_delay",
            default_value="5.0",
            description="Seconds the robot waits at the start"
        ),

        DeclareLaunchArgument(
            "step_period",
            default_value="2.0",
            description="Seconds per move along the path"
        ),

        # BVI planner + robot navigation
        Node(
            package="arp_assignment",
            executable="navigation_node",
            name="bvi_navigation",
            output="screen",
            parameters=[{
                "start_delay": start_delay,
                "step_period": step_period,
            }],
        ),

        # RViz2 with saved display configuration
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
