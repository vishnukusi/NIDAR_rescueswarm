import csv
import time
from pymavlink import mavutil


# ============================================================
# DOXI TELEMETRY LOGGER
# ============================================================

MAVLINK_CONNECTION = "udp:127.0.0.1:14550"

LOG_FILE = "drone_telemetry.csv"

LOG_INTERVAL = 0.1   # 10 samples per second


# ============================================================
# CONNECT TO MAVPROXY
# ============================================================

print()
print("=" * 65)
print("DOXI TELEMETRY LOGGER")
print("=" * 65)

print()
print("Connecting to MAVProxy...")

master = mavutil.mavlink_connection(
    MAVLINK_CONNECTION
)

print("Waiting for heartbeat...")

master.wait_heartbeat()

print(
    f"Connected to vehicle "
    f"{master.target_system}:{master.target_component}"
)


# ============================================================
# OPEN CSV
# ============================================================

csv_file = open(
    LOG_FILE,
    "w",
    newline=""
)

writer = csv.writer(csv_file)


writer.writerow([
    "time",
    "latitude",
    "longitude",
    "altitude",
    "relative_altitude",
    "velocity_x",
    "velocity_y",
    "velocity_z",
    "groundspeed",
    "heading"
])


print()
print(f"Logging telemetry to: {LOG_FILE}")
print("Press Ctrl+C to stop.")
print()


# ============================================================
# START LOGGING
# ============================================================

start_time = time.time()

last_log_time = 0


try:

    while True:

        current_time = time.time()

        # ----------------------------------------------------
        # Maintain logging rate
        # ----------------------------------------------------

        if current_time - last_log_time < LOG_INTERVAL:

            time.sleep(0.01)

            continue

        last_log_time = current_time


        # ====================================================
        # READ POSITION
        # ====================================================

        position = master.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=False
        )


        # ====================================================
        # READ VELOCITY / HEADING
        # ====================================================

        hud = master.recv_match(
            type="VFR_HUD",
            blocking=False
        )


        # ====================================================
        # DEFAULT VALUES
        # ====================================================

        latitude = ""
        longitude = ""
        altitude = ""

        relative_altitude = ""

        velocity_x = ""
        velocity_y = ""
        velocity_z = ""

        groundspeed = ""
        heading = ""


        # ====================================================
        # POSITION DATA
        # ====================================================

        if position is not None:

            latitude = position.lat / 1e7

            longitude = position.lon / 1e7

            altitude = position.alt / 1000.0

            relative_altitude = (
                position.relative_alt / 1000.0
            )

            velocity_x = position.vx / 100.0

            velocity_y = position.vy / 100.0

            velocity_z = position.vz / 100.0

            heading = position.hdg / 100.0


        # ====================================================
        # HUD DATA
        # ====================================================

        if hud is not None:

            groundspeed = hud.groundspeed

            if heading == "":

                heading = hud.heading


        # ====================================================
        # WRITE ROW
        # ====================================================

        elapsed = current_time - start_time

        writer.writerow([

            f"{elapsed:.2f}",

            latitude,

            longitude,

            altitude,

            relative_altitude,

            velocity_x,

            velocity_y,

            velocity_z,

            groundspeed,

            heading

        ])


        csv_file.flush()


        # ====================================================
        # TERMINAL OUTPUT
        # ====================================================

        print(

            f"\rTime: {elapsed:6.1f}s | "

            f"Lat: {latitude} | "

            f"Lon: {longitude} | "

            f"Alt: {relative_altitude}m | "

            f"Speed: {groundspeed}",

            end=""

        )


except KeyboardInterrupt:

    print()
    print()
    print("Stopping telemetry logger...")


finally:

    csv_file.close()

    master.close()


print()
print("=" * 65)
print("TELEMETRY LOGGING FINISHED")
print("=" * 65)

print(f"Saved: {LOG_FILE}")