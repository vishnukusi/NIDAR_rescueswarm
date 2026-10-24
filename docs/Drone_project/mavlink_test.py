from pymavlink import mavutil
import time

print("Connecting to ArduCopter SITL...")

connection = mavutil.mavlink_connection(
    "udp:127.0.0.1:14550"
)

print("Waiting for heartbeat...")

connection.wait_heartbeat()

print("\nCONNECTED TO VIRTUAL DRONE!")

print("System ID:", connection.target_system)
print("Component ID:", connection.target_component)

print("\nReading telemetry...\n")

for i in range(10):

    msg = connection.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=True,
        timeout=2
    )

    if msg:

        latitude = msg.lat / 1e7
        longitude = msg.lon / 1e7
        altitude = msg.relative_alt / 1000.0

        print(
            f"Latitude: {latitude:.7f} | "
            f"Longitude: {longitude:.7f} | "
            f"Altitude: {altitude:.2f} m"
        )

    time.sleep(0.5)

print("\nMAVLink test completed.")