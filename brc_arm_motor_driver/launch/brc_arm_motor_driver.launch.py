import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("brc_arm_motor_driver"),
        "config",
        "motor_config.yaml",
    )

    node = Node(
        package="brc_arm_motor_driver",
        name="brc_arm_motor_driver",
        executable="brc_arm_motor_driver",
        output="screen",
        emulate_tty=True,
    )
    ld.add_action(node)
    return ld
