"""Launch file for PX4 SITL multi-vehicle transport bridge.

Starts micro-XRCE-DDS Agent or MAVLink routing endpoints for S1-S3 and D1-D3.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "xrce_port",
        default_value="8888",
        description="UDP port for MicroXRCEAgent (default: 8888)",
    )

    # Micro XRCE-DDS Agent process
    xrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", LaunchConfiguration("xrce_port")],
        output="screen",
    )

    return LaunchDescription([port_arg, xrce_agent])
