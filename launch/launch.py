# Copyright 2026 PIB Robotics Team

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="hand_nucleo",
                executable="hand_nucleo_bridge",
                namespace="hand/nucleo",
                parameters=[
                    {
                        "serial_port": os.getenv(
                            "NUCLEO_SERIAL_PORT", "/dev/ttyACM0"
                        ),
                    }
                ],
            )
        ]
    )
