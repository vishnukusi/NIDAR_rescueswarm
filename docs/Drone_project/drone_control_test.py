from pymavlink import mavutil
import time


# ============================================================
# CONNECT TO ARDUCOPTER SITL THROUGH MAVPROXY
# ============================================================

print("Connecting to virtual drone...")

master = mavutil.mavlink_connection(
    "udp:127.0.0.1:14550"
)

print("Waiting for heartbeat...")

master.wait_heartbeat()

print("CONNECTED!")
print(
    f"System ID: {master.target_system}, "
    f"Component ID: {master.target_component}"
)


# ============================================================
# SET GUIDED MODE
# ============================================================

print("\nChanging mode to GUIDED...")

mode = "GUIDED"

mode_id = master.mode_mapping().get(mode)

if mode_id is None:
    print("ERROR: GUIDED mode not available.")
    master.close()
    exit()

master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)

time.sleep(2)

print("GUIDED mode command sent.")


# ============================================================
# ARM THE VIRTUAL DRONE
# ============================================================

print("\nArming virtual drone...")

master.arducopter_arm()

master.motors_armed_wait()

print("VIRTUAL DRONE ARMED!")


# ============================================================
# SEND VELOCITY COMMAND
# ============================================================

def send_velocity(vx, vy, vz):

    """
    Send velocity in LOCAL NED frame.

    vx = North/South velocity
    vy = East/West velocity
    vz = Down/Up velocity

    Positive vz = DOWN
    Negative vz = UP
    """

    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,

        mavutil.mavlink.MAV_FRAME_LOCAL_NED,

        0b0000111111000111,

        0, 0, 0,

        vx, vy, vz,

        0, 0, 0,

        0, 0
    )


# ============================================================
# TEST MOVEMENT
# ============================================================

print("\nSending RIGHT/EAST movement...")

print("Virtual drone will move for 3 seconds.")

for i in range(30):

    # 0.5 m/s East
    send_velocity(
        0,
        0.5,
        0
    )

    time.sleep(0.1)


# ============================================================
# STOP
# ============================================================

print("\nStopping movement...")

for i in range(10):

    send_velocity(
        0,
        0,
        0
    )

    time.sleep(0.1)


print("Movement command finished.")


# ============================================================
# DISARM
# ============================================================

print("\nDisarming...")

master.arducopter_disarm()

time.sleep(2)

print("Test completed.")

master.close()