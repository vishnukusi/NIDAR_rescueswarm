"""Launch file for NIDAR RescueSwarm 6-Drone Mission Autonomy Stack.

Launches:
- 1 x MissionManagerNode (coordinates full 6-drone fleet)
- 1 x GCSBridgeNode (bridges telemetry to GCS web UI)
- 3 x PerceptionBridgeNode (S1, S2, S3 camera perception & localization)
- 3 x SimulatedDeliveryNode (D1, D2, D3 simulated drop verification)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = os.path.dirname(os.path.dirname(__file__))

    adapter_arg = DeclareLaunchArgument(
        "adapter_type",
        default_value="px4_mavlink",
        description="Vehicle adapter type: mock, px4_ros2, or px4_mavlink",
    )
    config_arg = DeclareLaunchArgument(
        "fleet_config",
        default_value="config/fleet.json",
        description="Path to fleet JSON configuration file",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_share, "config", "ros2_params.yaml"),
        description="Path to ROS 2 parameters YAML file",
    )

    nodes = [
        # 1. Global Fleet Mission Manager
        Node(
            package="rescue_swarm_bridge",
            executable="mission_manager_node",
            name="mission_manager_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "config_path": LaunchConfiguration("fleet_config"),
                    "adapter_type": LaunchConfiguration("adapter_type"),
                    "timeout_s": 300.0,
                },
            ],
        ),
        # 2. GCS Telemetry Bridge
        Node(
            package="rescue_swarm_bridge",
            executable="gcs_bridge_node",
            name="gcs_bridge_node",
            output="screen",
        ),
        # 3. Search Drone Perception Nodes (S1, S2, S3)
        Node(
            package="rescue_swarm_bridge",
            executable="perception_bridge_node",
            name="perception_bridge_s1",
            namespace="s1",
            parameters=[{"drone_id": "S1"}],
            output="screen",
        ),
        Node(
            package="rescue_swarm_bridge",
            executable="perception_bridge_node",
            name="perception_bridge_s2",
            namespace="s2",
            parameters=[{"drone_id": "S2"}],
            output="screen",
        ),
        Node(
            package="rescue_swarm_bridge",
            executable="perception_bridge_node",
            name="perception_bridge_s3",
            namespace="s3",
            parameters=[{"drone_id": "S3"}],
            output="screen",
        ),
        # 4. Delivery Drone Simulation Verification Nodes (D1, D2, D3)
        Node(
            package="rescue_swarm_bridge",
            executable="simulated_delivery_node",
            name="simulated_delivery_d1",
            namespace="d1",
            parameters=[{"drone_id": "D1", "verification_mode": "proximity_and_hover"}],
            output="screen",
        ),
        Node(
            package="rescue_swarm_bridge",
            executable="simulated_delivery_node",
            name="simulated_delivery_d2",
            namespace="d2",
            parameters=[{"drone_id": "D2", "verification_mode": "proximity_and_hover"}],
            output="screen",
        ),
        Node(
            package="rescue_swarm_bridge",
            executable="simulated_delivery_node",
            name="simulated_delivery_d3",
            namespace="d3",
            parameters=[{"drone_id": "D3", "verification_mode": "proximity_and_hover"}],
            output="screen",
        ),
    ]

    return LaunchDescription([adapter_arg, config_arg, params_arg] + nodes)
