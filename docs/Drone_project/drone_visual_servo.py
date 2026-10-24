import cv2
import time
from ultralytics import YOLO
from pymavlink import mavutil


# ============================================================
# DOXI AI - LAPTOP CAMERA VISUAL SERVOING
# best.pt -> MAVLink -> MAVProxy -> ArduCopter SITL
#
# Goal:
#   Detect person/pedestrian with laptop camera
#   Calculate image error from camera center
#   Translate simulated drone toward target
#   DO NOT intentionally rotate the drone
# ============================================================


MODEL_PATH = "yolov8n.pt"

CAMERA_INDEX = 0

# Target takeoff height in meters
TARGET_ALTITUDE = 10.0

# Python receives MAVLink from MAVProxy on 14550
MAVLINK_ADDRESS = "udp:127.0.0.1:14550"


# ============================================================
# YOLO SETTINGS
# ============================================================

CONFIDENCE = 0.35

# Detect both classes:
# 0 = person
# 1 = pedestrian
DETECTION_CLASSES = [0, 1]


# ============================================================
# CENTERING SETTINGS
# ============================================================

DEADBAND_X = 40
DEADBAND_Y = 40


# ============================================================
# MOVEMENT CONTROL
# ============================================================

KP_X = 0.006
KP_Y = 0.006

MAX_SPEED = 1.0

SMOOTHING = 0.30


# ============================================================
# CONNECT TO MAVPROXY
# ============================================================

print()
print("=" * 70)
print("DOXI AI -> MAVLINK VISUAL SERVOING")
print("=" * 70)

print()
print("Connecting to MAVProxy...")

vehicle = mavutil.mavlink_connection(
    MAVLINK_ADDRESS
)

print("Waiting for heartbeat...")

vehicle.wait_heartbeat()

print(
    f"Connected to vehicle "
    f"{vehicle.target_system}:{vehicle.target_component}"
)


# ============================================================
# SET GUIDED MODE
# ============================================================

print("Setting GUIDED mode...")

vehicle.set_mode_apm("GUIDED")

time.sleep(2)

print("GUIDED mode requested.")


# ============================================================
# ARM THE DRONE
# ============================================================

print()
print("Arming drone...")

vehicle.arducopter_arm()
vehicle.motors_armed_wait()

print("DRONE ARMED SUCCESSFULLY!")


# ============================================================
# TAKEOFF TO 10M HEIGHT
# ============================================================

print()
print(f"Commanding takeoff to {TARGET_ALTITUDE}m altitude...")

vehicle.mav.command_long_send(
    vehicle.target_system,
    vehicle.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0,
    0, 0,
    TARGET_ALTITUDE
)

print(f"Flying to {TARGET_ALTITUDE}m height...")

while True:
    message = vehicle.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
    if message is not None:
        relative_alt = message.relative_alt / 1000.0
        print(f"Current Altitude: {relative_alt:.2f} m")
        if relative_alt >= TARGET_ALTITUDE * 0.95:
            print(f"Target height of {TARGET_ALTITUDE}m reached ({relative_alt:.2f}m)!")
            break
    else:
        print("Waiting for altitude telemetry...")
    time.sleep(0.5)


# ============================================================
# LOAD CUSTOM YOLO MODEL
# ============================================================

print()
print("Loading custom YOLO model (best.pt)...")

model = YOLO(MODEL_PATH)

print("Custom YOLO model loaded successfully.")
print("Model classes:", model.names)


# ============================================================
# OPEN LAPTOP CAMERA & START PERSON CHECKING
# ============================================================

print()
print("Opening laptop camera feed...")

camera = cv2.VideoCapture(CAMERA_INDEX)

if not camera.isOpened():

    print("ERROR: Could not open laptop camera.")

    vehicle.close()

    raise SystemExit


print("Camera opened successfully.")
print("Starting person detection and visual servoing from live camera feed...")
print("Move yourself around the camera.")
print("Press Q to stop.")
print()


# ============================================================
# GET CURRENT YAW
# ============================================================

def get_current_yaw():

    """
    Get current vehicle yaw in radians.

    MAVLink ATTITUDE yaw is already expressed in radians.
    """

    message = vehicle.recv_match(
        type="ATTITUDE",
        blocking=False
    )

    if message is not None:

        return float(message.yaw)

    return 0.0


# ============================================================
# SEND LOCAL-NED VELOCITY
# ============================================================

def send_velocity(forward, right, hold_yaw):

    """
    Send TRANSLATIONAL velocity in LOCAL_NED.

    LOCAL_NED:

        X = North
        Y = East
        Z = Down

    We explicitly provide yaw and yaw-rate.

    yaw       = current heading
    yaw_rate  = 0

    Therefore the controller does not intentionally rotate
    the vehicle.

    forward and right are converted from the camera/body
    interpretation into local North/East using the current yaw.
    """

    # --------------------------------------------------------
    # Convert body-frame movement into local N/E movement
    # --------------------------------------------------------

    north_velocity = (
        forward * __import__("math").cos(hold_yaw)
        -
        right * __import__("math").sin(hold_yaw)
    )

    east_velocity = (
        forward * __import__("math").sin(hold_yaw)
        +
        right * __import__("math").cos(hold_yaw)
    )


    # --------------------------------------------------------
    # Type mask
    #
    # Use:
    #   velocity X/Y/Z
    #   yaw
    #
    # Ignore:
    #   position
    #   acceleration
    #   yaw rate
    # --------------------------------------------------------

    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
        |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
        |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
        |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )


    # --------------------------------------------------------
    # Send command
    # --------------------------------------------------------

    vehicle.mav.set_position_target_local_ned_send(

        0,

        vehicle.target_system,
        vehicle.target_component,

        mavutil.mavlink.MAV_FRAME_LOCAL_NED,

        type_mask,

        # Position - ignored
        0,
        0,
        0,

        # Velocity - USED
        float(north_velocity),
        float(east_velocity),
        0.0,

        # Acceleration - ignored
        0.0,
        0.0,
        0.0,

        # Keep current yaw
        float(hold_yaw),

        # No yaw rate
        0.0
    )


# ============================================================
# STOP DRONE
# ============================================================

def stop_drone():

    yaw = get_current_yaw()

    send_velocity(
        0.0,
        0.0,
        yaw
    )


# ============================================================
# INITIAL YAW
# ============================================================

print()
print("Reading current drone heading...")

current_yaw = get_current_yaw()

print(
    f"Current yaw: "
    f"{current_yaw:.3f} radians"
)

print()
print("Yaw lock enabled.")
print("Visual controller will command translation only.")
print()


# ============================================================
# SMOOTHING VARIABLES
# ============================================================

smooth_x = None
smooth_y = None


# ============================================================
# STATISTICS
# ============================================================

frame_count = 0
detection_count = 0
centered_count = 0

start_time = time.time()


# ============================================================
# MAIN LOOP
# ============================================================

try:

    while True:

        ret, frame = camera.read()

        if not ret:

            print("Camera frame failed.")

            stop_drone()

            break


        frame_count += 1


        # ----------------------------------------------------
        # FRAME SIZE
        # ----------------------------------------------------

        height, width = frame.shape[:2]

        camera_x = width // 2
        camera_y = height // 2


        # ----------------------------------------------------
        # YOLO DETECTION
        # ----------------------------------------------------

        result = model(

            frame,

            imgsz=640,

            conf=CONFIDENCE,

            classes=DETECTION_CLASSES,

            max_det=1,

            verbose=False

        )[0]


        # ----------------------------------------------------
        # CAMERA CENTER
        # ----------------------------------------------------

        cv2.circle(

            frame,

            (camera_x, camera_y),

            8,

            (255, 0, 0),

            -1

        )

        cv2.putText(

            frame,

            "DRONE CENTER",

            (camera_x + 10, camera_y - 10),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.5,

            (255, 0, 0),

            2

        )


        # ====================================================
        # TARGET DETECTED
        # ====================================================

        if len(result.boxes) > 0:

            detection_count += 1

            box = result.boxes[0]


            # ------------------------------------------------
            # BOUNDING BOX
            # ------------------------------------------------

            x1, y1, x2, y2 = map(

                int,

                box.xyxy[0]

            )


            confidence = float(

                box.conf[0]

            )


            class_id = int(

                box.cls[0]

            )


            class_name = model.names[class_id]


            # ------------------------------------------------
            # TARGET CENTER
            # ------------------------------------------------

            target_raw_x = int(

                (x1 + x2) / 2

            )

            target_raw_y = int(

                (y1 + y2) / 2

            )


            # ------------------------------------------------
            # SMOOTH TARGET
            # ------------------------------------------------

            if smooth_x is None:

                smooth_x = float(target_raw_x)

                smooth_y = float(target_raw_y)

            else:

                smooth_x = (

                    SMOOTHING * target_raw_x

                    +

                    (1 - SMOOTHING) * smooth_x

                )

                smooth_y = (

                    SMOOTHING * target_raw_y

                    +

                    (1 - SMOOTHING) * smooth_y

                )


            target_x = int(smooth_x)

            target_y = int(smooth_y)


            # ------------------------------------------------
            # IMAGE ERROR
            # ------------------------------------------------

            error_x = target_x - camera_x

            error_y = target_y - camera_y


            # ------------------------------------------------
            # PROPORTIONAL CONTROL
            # ------------------------------------------------

            right_velocity = KP_X * error_x

            forward_velocity = -KP_Y * error_y


            # ------------------------------------------------
            # DEAD BAND
            # ------------------------------------------------

            if abs(error_x) <= DEADBAND_X:

                right_velocity = 0.0


            if abs(error_y) <= DEADBAND_Y:

                forward_velocity = 0.0


            # ------------------------------------------------
            # LIMIT SPEED
            # ------------------------------------------------

            right_velocity = max(

                -MAX_SPEED,

                min(

                    MAX_SPEED,

                    right_velocity

                )

            )


            forward_velocity = max(

                -MAX_SPEED,

                min(

                    MAX_SPEED,

                    forward_velocity

                )

            )


            # ------------------------------------------------
            # COMMAND DESCRIPTION
            # ------------------------------------------------

            commands = []


            if right_velocity > 0:

                commands.append("RIGHT")

            elif right_velocity < 0:

                commands.append("LEFT")


            if forward_velocity > 0:

                commands.append("FORWARD")

            elif forward_velocity < 0:

                commands.append("BACKWARD")


            if len(commands) == 0:

                command = "HOLD POSITION"

                centered_count += 1

            else:

                command = " + ".join(commands)


            # ------------------------------------------------
            # UPDATE CURRENT YAW
            #
            # We track yaw but don't command a rotation.
            # ------------------------------------------------

            yaw_message = vehicle.recv_match(

                type="ATTITUDE",

                blocking=False

            )


            if yaw_message is not None:

                current_yaw = float(

                    yaw_message.yaw

                )


            # ------------------------------------------------
            # SEND TRANSLATION COMMAND
            # ------------------------------------------------

            send_velocity(

                forward_velocity,

                right_velocity,

                current_yaw

            )


            # ------------------------------------------------
            # DRAW DETECTION
            # ------------------------------------------------

            cv2.rectangle(

                frame,

                (x1, y1),

                (x2, y2),

                (0, 255, 0),

                2

            )


            # ------------------------------------------------
            # RAW CENTER
            # ------------------------------------------------

            cv2.circle(

                frame,

                (target_raw_x, target_raw_y),

                6,

                (0, 255, 255),

                -1

            )


            # ------------------------------------------------
            # SMOOTH CENTER
            # ------------------------------------------------

            cv2.circle(

                frame,

                (target_x, target_y),

                10,

                (0, 0, 255),

                -1

            )


            # ------------------------------------------------
            # ERROR VECTOR
            # ------------------------------------------------

            cv2.arrowedLine(

                frame,

                (camera_x, camera_y),

                (target_x, target_y),

                (0, 255, 255),

                3

            )


            # =================================================
            # DISPLAY
            # =================================================

            cv2.putText(

                frame,

                f"TARGET: {class_name} "

                f"({confidence:.2f})",

                (20, 35),

                cv2.FONT_HERSHEY_SIMPLEX,

                0.60,

                (0, 255, 0),

                2

            )


            cv2.putText(

                frame,

                f"ERROR X: {error_x:+d} px",

                (20, 70),

                cv2.FONT_HERSHEY_SIMPLEX,

                0.60,

                (255, 255, 255),

                2

            )


            cv2.putText(

                frame,

                f"ERROR Y: {error_y:+d} px",

                (20, 105),

                cv2.FONT_HERSHEY_SIMPLEX,

                0.60,

                (255, 255, 255),

                2

            )


            cv2.putText(

                frame,

                f"FORWARD: "

                f"{forward_velocity:+.2f} m/s",

                (20, 140),

                cv2.FONT_HERSHEY_SIMPLEX,

                0.55,

                (255, 255, 255),

                2

            )


            cv2.putText(

                frame,

                f"RIGHT: "

                f"{right_velocity:+.2f} m/s",

                (20, 175),

                cv2.FONT_HERSHEY_SIMPLEX,

                0.55,

                (255, 255, 255),

                2

            )


            cv2.putText(

                frame,

                f"COMMAND: {command}",

                (20, 215),

                cv2.FONT_HERSHEY_SIMPLEX,

                0.65,

                (0, 255, 255),

                2

            )


            cv2.putText(

                frame,

                "YAW: LOCKED",

                (20, 250),

                cv2.FONT_HERSHEY_SIMPLEX,

                0.60,

                (255, 255, 255),

                2

            )


        # ====================================================
        # NO TARGET
        # ====================================================

        else:

            stop_drone()

            cv2.putText(

                frame,

                "NO PERSON / PEDESTRIAN",

                (20, 40),

                cv2.FONT_HERSHEY_SIMPLEX,

                0.65,

                (0, 0, 255),

                2

            )


            cv2.putText(

                frame,

                "COMMAND: HOLD",

                (20, 80),

                cv2.FONT_HERSHEY_SIMPLEX,

                0.65,

                (0, 0, 255),

                2

            )


        # ====================================================
        # FPS
        # ====================================================

        elapsed = time.time() - start_time

        if elapsed > 0:

            fps = frame_count / elapsed

        else:

            fps = 0


        cv2.putText(

            frame,

            f"FPS: {fps:.1f}",

            (width - 120, 35),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.55,

            (255, 255, 255),

            2

        )


        # ====================================================
        # SHOW CAMERA
        # ====================================================

        cv2.imshow(

            "DOXI AI - Visual Servoing",

            frame

        )


        # ====================================================
        # QUIT
        # ====================================================

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):

            break


# ============================================================
# INTERRUPT
# ============================================================

except KeyboardInterrupt:

    print()
    print("Experiment interrupted.")


# ============================================================
# SAFE SHUTDOWN
# ============================================================

finally:

    print()
    print("Stopping drone movement...")

    stop_drone()

    time.sleep(0.5)

    if 'camera' in locals() and camera is not None:
        camera.release()

    cv2.destroyAllWindows()

    vehicle.close()


# ============================================================
# RESULTS
# ============================================================

print()
print("=" * 70)
print("DOXI VISUAL SERVOING FINISHED")
print("=" * 70)

print(
    f"Frames processed : "
    f"{frame_count}"
)

print(
    f"Detections       : "
    f"{detection_count}"
)

print(
    f"Centered frames  : "
    f"{centered_count}"
)

if frame_count > 0:

    detection_rate = (

        detection_count
        /
        frame_count

    ) * 100

    print(
        f"Detection rate   : "
        f"{detection_rate:.2f}%"
    )

print()
print("Experiment complete.")