"""ROS 2 Standalone Vehicle Adapter Node for NIDAR RescueSwarm.

Provides a dedicated ROS 2 node interface for a single vehicle in the swarm.
Exposes telemetry, flight mode, and command interfaces while bridging to PX4 or Mock backends.
"""

import json
import logging
import os
import sys
import time
from typing import Any, Dict, List, Optional

from .vehicle_adapter import (
    AdapterType,
    VehicleAdapter,
    create_vehicle_adapter,
)

logger = logging.getLogger(__name__)


def main(args: Optional[List[str]] = None) -> None:
    """ROS 2 Node Entry Point."""
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        rclpy.init(args=args)

        class VehicleAdapterNode(Node):
            def __init__(self) -> None:
                super().__init__("vehicle_adapter_node")
                self.declare_parameter("drone_id", "S1")
                self.declare_parameter("adapter_type", "mock")
                self.declare_parameter("mavlink_port", 14540)
                self.declare_parameter("system_id", 1)

                drone_id = self.get_parameter("drone_id").get_parameter_value().string_value
                adapter_type_str = self.get_parameter("adapter_type").get_parameter_value().string_value
                port = self.get_parameter("mavlink_port").get_parameter_value().integer_value
                sysid = self.get_parameter("system_id").get_parameter_value().integer_value

                cfg = {
                    "drone_id": drone_id,
                    "adapter_type": adapter_type_str,
                    "mavlink_port": port,
                    "system_id": sysid,
                    "ros_namespace": drone_id.lower(),
                }
                ad_type = AdapterType(adapter_type_str)
                self.adapter = create_vehicle_adapter(
                    drone_id=drone_id,
                    config=cfg,
                    adapter_type=ad_type,
                    ros2_node=self,
                )

                self.pub_telemetry = self.create_publisher(
                    String, f"/{drone_id.lower()}/telemetry", 10
                )
                self.sub_command = self.create_subscription(
                    String, f"/{drone_id.lower()}/command", self.command_callback, 10
                )
                self.sub_setpoint = self.create_subscription(
                    String, f"/{drone_id.lower()}/setpoint", self.setpoint_callback, 10
                )
                self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
                self.adapter.connect()
                self.get_logger().info(f"VehicleAdapterNode started for {drone_id} (backend: {ad_type.value})")

            def command_callback(self, msg: Any) -> None:
                try:
                    cmd_data = json.loads(msg.data)
                    cmd_type = cmd_data.get("command", "").upper()
                    if cmd_type == "ARM":
                        self.adapter.arm()
                    elif cmd_type == "DISARM":
                        self.adapter.disarm()
                    elif cmd_type == "SET_MODE":
                        mode = cmd_data.get("mode", "HOLD")
                        self.adapter.set_mode(mode)
                    elif cmd_type in ("HOLD", "RTL", "LAND", "EMERGENCY_LAND"):
                        self.adapter.send_safety_action(cmd_type)
                except Exception as e:
                    self.get_logger().error(f"Failed to process command: {e}")

            def setpoint_callback(self, msg: Any) -> None:
                try:
                    sp_data = json.loads(msg.data)
                    lat = float(sp_data["lat"])
                    lon = float(sp_data["lon"])
                    alt = float(sp_data.get("alt", 25.0))
                    yaw = float(sp_data.get("yaw", 0.0))
                    self.adapter.send_position_setpoint(lat, lon, alt, yaw)
                except Exception as e:
                    self.get_logger().error(f"Failed to process setpoint: {e}")

            def timer_callback(self) -> None:
                telem = self.adapter.update()
                if telem:
                    msg = String()
                    msg.data = json.dumps({
                        "drone_id": telem.drone_id,
                        "timestamp": telem.timestamp,
                        "connected": telem.connected,
                        "armed": telem.armed,
                        "mode": telem.mode,
                        "lat": telem.latitude,
                        "lon": telem.longitude,
                        "alt_agl": telem.altitude_agl_m,
                        "battery": telem.battery_fraction,
                        "is_mock": telem.is_mock,
                    })
                    self.pub_telemetry.publish(msg)

        node = VehicleAdapterNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except ImportError:
        logger.warning("rclpy not installed; VehicleAdapterNode running in standalone mode.")


if __name__ == "__main__":
    main()
