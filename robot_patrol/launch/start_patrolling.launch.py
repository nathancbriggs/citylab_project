from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare("robot_patrol"),
        "rviz",
        "patrol.rviz",
    ])

    return LaunchDescription([
        Node(
            package="robot_patrol",
            executable="patrol",
            name="patrol",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
    ])