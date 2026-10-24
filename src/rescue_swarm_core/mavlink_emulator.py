"""Lightweight MAVLink SITL loopback emulator for unit and integration testing.

Emulates an ArduCopter / PX4 autopilot over real UDP sockets, emitting standard
binary MAVLink HEARTBEAT, GLOBAL_POSITION_INT, and ATTITUDE packets and responding
to ARM, TAKEOFF, SET_POSITION_TARGET_LOCAL_NED velocity commands, and safety overrides.
"""

import math
import socket
import threading
import time
from typing import Optional, Tuple

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2


class MockSITLEmulator:
    """Local UDP MAVLink server emulating drone flight dynamics and autopilot telemetry."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 14550,
        system_id: int = 1,
        component_id: int = 1,
        initial_lat: float = 12.972000,
        initial_lon: float = 77.594500,
        initial_alt_m: float = 0.0,
        initial_yaw_rad: float = 0.7854,  # 45 deg NE
    ) -> None:
        self.host = host
        self.port = port
        self.system_id = system_id
        self.component_id = component_id

        # Vehicle flight state
        self.lat = initial_lat
        self.lon = initial_lon
        self.alt_agl_m = initial_alt_m
        self.alt_msl_m = initial_alt_m + 920.0  # ~Bangalore elevation
        self.yaw_rad = initial_yaw_rad
        self.pitch_rad = 0.0
        self.roll_rad = 0.0
        self.vel_n_mps = 0.0
        self.vel_e_mps = 0.0
        self.vel_d_mps = 0.0
        self.battery_remaining = 94

        self.armed = False
        self.mode = "STABILIZE"
        self.target_alt_m = 0.0
        self.is_running = False

        self._thread: Optional[threading.Thread] = None
        self._sock: Optional[socket.socket] = None
        self._client_addr: Optional[Tuple[str, int]] = None
        self._mav: Optional[mavlink2.MAVLink] = None

    def start(self) -> None:
        """Start emulator UDP listening and telemetry broadcast thread."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.settimeout(0.05)

        class SockWriter:
            def __init__(self, sock, get_client):
                self.sock = sock
                self.get_client = get_client

            def write(self, data):
                client = self.get_client()
                if client:
                    try:
                        self.sock.sendto(data, client)
                    except Exception:
                        pass

        self._mav = mavlink2.MAVLink(
            SockWriter(self._sock, lambda: self._client_addr),
            srcSystem=self.system_id,
            srcComponent=self.component_id,
        )

        self.is_running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop emulator loop and close socket."""
        self.is_running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def _run_loop(self) -> None:
        """Main loop: parse incoming MAVLink packets and broadcast telemetry at 10 Hz."""
        mav_parser = mavlink2.MAVLink(None)
        last_telem_time = 0.0

        while self.is_running:
            now = time.time()
            dt = 0.05

            # 1. Ingest client packets
            try:
                data, addr = self._sock.recvfrom(2048)
                self._client_addr = addr
                msgs = mav_parser.parse_buffer(data)
                if msgs:
                    for msg in msgs:
                        self._handle_message(msg)
            except socket.timeout:
                pass
            except Exception:
                pass

            # 2. Integrate vehicle kinematics
            if self.armed and self.alt_agl_m > 0.5:
                # Update positions based on commanded velocity
                meters_to_lat = 1.0 / 111139.0
                meters_to_lon = 1.0 / (111139.0 * math.cos(math.radians(self.lat)))
                self.lat += self.vel_n_mps * dt * meters_to_lat
                self.lon += self.vel_e_mps * dt * meters_to_lon

            # Handle takeoff climb
            if self.armed and self.alt_agl_m < self.target_alt_m:
                self.alt_agl_m = min(self.target_alt_m, self.alt_agl_m + 3.0 * dt)
                self.alt_msl_m = self.alt_agl_m + 920.0

            # 3. Stream Telemetry at 10 Hz
            if now - last_telem_time >= 0.10:
                last_telem_time = now
                self._send_heartbeat()
                self._send_global_position()
                self._send_attitude()

            time.sleep(dt)

    def _handle_message(self, msg) -> None:
        """Process incoming MAVLink commands from client bridge."""
        msg_type = msg.get_type()

        if msg_type == "COMMAND_LONG":
            cmd = msg.command
            if cmd == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                self.armed = (msg.param1 == 1.0)
            elif cmd == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                self.target_alt_m = float(msg.param7)
            elif cmd == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                self.mode = "RTL"
                self.vel_n_mps = 0.0
                self.vel_e_mps = 0.0
            elif cmd == mavutil.mavlink.MAV_CMD_NAV_LAND:
                self.mode = "LAND"
                self.vel_n_mps = 0.0
                self.vel_e_mps = 0.0

        elif msg_type == "SET_MODE":
            self.mode = "GUIDED"

        elif msg_type == "SET_POSITION_TARGET_LOCAL_NED":
            self.vel_n_mps = float(msg.vx)
            self.vel_e_mps = float(msg.vy)
            self.vel_d_mps = float(msg.vz)
            if not (msg.type_mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE):
                self.yaw_rad = float(msg.yaw)

    def _send_heartbeat(self) -> None:
        if not self._mav:
            return
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if self.armed:
            base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        custom_mode = 4 if self.mode == "GUIDED" else 0  # 4 = APM GUIDED
        self._mav.heartbeat_send(
            type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOT,
            base_mode=base_mode,
            custom_mode=custom_mode,
            system_status=mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def _send_global_position(self) -> None:
        if not self._mav:
            return
        hdg_cdeg = int((math.degrees(self.yaw_rad) % 360.0) * 100)
        self._mav.global_position_int_send(
            time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
            lat=int(self.lat * 1e7),
            lon=int(self.lon * 1e7),
            alt=int(self.alt_msl_m * 1e3),
            relative_alt=int(self.alt_agl_m * 1e3),
            vx=int(self.vel_n_mps * 100),
            vy=int(self.vel_e_mps * 100),
            vz=int(self.vel_d_mps * 100),
            hdg=hdg_cdeg,
        )

    def _send_attitude(self) -> None:
        if not self._mav:
            return
        self._mav.attitude_send(
            time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
            roll=float(self.roll_rad),
            pitch=float(self.pitch_rad),
            yaw=float(self.yaw_rad),
            rollspeed=0.0,
            pitchspeed=0.0,
            yawspeed=0.0,
        )
