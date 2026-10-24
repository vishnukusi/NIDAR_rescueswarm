from pymavlink import mavutil
import time

print("Connecting to MAVProxy...")

vehicle = mavutil.mavlink_connection(
    "udp:127.0.0.1:14550"
)

print("Waiting for heartbeat...")

vehicle.wait_heartbeat()

print("CONNECTED!")
print(
    "System:",
    vehicle.target_system,
    "Component:",
    vehicle.target_component
)

print("Sending RIGHT command for 5 seconds...")

for i in range(50):

    vehicle.mav.set_position_target_local_ned_send(
        0,
        vehicle.target_system,
        vehicle.target_component,

        mavutil.mavlink.MAV_FRAME_BODY_NED,

        0b0000111111000111,

        0, 0, 0,

        0,      # forward
        1.0,    # right
        0,      # down

        0, 0, 0,

        0, 0
    )

    time.sleep(0.1)

print("STOPPING...")

vehicle.mav.set_position_target_local_ned_send(
    0,
    vehicle.target_system,
    vehicle.target_component,

    mavutil.mavlink.MAV_FRAME_BODY_NED,

    0b0000111111000111,

    0, 0, 0,

    0,
    0,
    0,

    0, 0, 0,

    0, 0
)

print("TEST FINISHED")

vehicle.close()