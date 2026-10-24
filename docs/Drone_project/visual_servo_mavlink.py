import cv2
import time
from ultralytics import YOLO
from pymavlink import mavutil


# ============================================================
# SETTINGS
# ============================================================

MODEL_PATH = "yolov8n.pt"
CAMERA_INDEX = 0

CONFIDENCE_THRESHOLD = 0.30

DEADBAND_X = 30
DEADBAND_Y = 30

TARGET_RATIO = 0.12

KP_FORWARD = 0.008
KP_RIGHT = 0.008

MAX_VELOCITY = 1.0

SMOOTHING = 0.25


# ============================================================
# MAVLINK CONNECTION
# ============================================================

print()
print("=" * 65)
print("DOXI BODY-FRAME VISUAL SERVOING")
print("=" * 65)

print()
print("Connecting to MAVProxy...")

master = mavutil.mavlink_connection(
    "udp:127.0.0.1:14550"
)

print("Waiting for heartbeat...")

master.wait_heartbeat()

print(
    f"Connected to vehicle "
    f"{master.target_system}:{master.target_component}"
)


# ============================================================
# LOAD YOLO
# ============================================================

print()
print("Loading YOLOv8n...")

model = YOLO(MODEL_PATH)

print("YOLOv8n loaded successfully.")


# ============================================================
# CAMERA
# ============================================================

print()
print("Opening laptop camera...")

cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():

    print("ERROR: Could not open camera.")

    master.close()

    raise SystemExit


print("Camera opened.")
print("Press Q to quit.")


# ============================================================
# TARGET STATE
# ============================================================

smoothed_x = None
smoothed_y = None

frame_count = 0
detection_count = 0

start_time = time.time()


# ============================================================
# SEND BODY-FRAME VELOCITY
# ============================================================

def send_body_velocity(forward, right, down):

    """
    Send velocity in the drone BODY_NED frame.

    forward:
        + = forward
        - = backward

    right:
        + = right
        - = left

    down:
        + = down
        - = up
    """

    type_mask = 0b0000111111000111

    master.mav.set_position_target_local_ned_send(

        0,

        master.target_system,
        master.target_component,

        mavutil.mavlink.MAV_FRAME_BODY_NED,

        type_mask,

        0,
        0,
        0,

        forward,
        right,
        down,

        0,
        0,
        0,

        0,
        0
    )


# ============================================================
# MAIN LOOP
# ============================================================

while True:

    ret, frame = cap.read()

    if not ret:

        print("ERROR: Camera frame unavailable.")

        break


    frame_count += 1

    height, width = frame.shape[:2]

    center_x = width // 2
    center_y = height // 2


    # ========================================================
    # YOLO DETECTION
    # ========================================================

    results = model(

        frame,

        imgsz=640,

        conf=CONFIDENCE_THRESHOLD,

        classes=[0],

        max_det=1,

        verbose=False

    )[0]


    # ========================================================
    # CAMERA CENTER
    # ========================================================

    cv2.circle(

        frame,

        (center_x, center_y),

        8,

        (255, 0, 0),

        -1
    )


    # ========================================================
    # PERSON FOUND
    # ========================================================

    if len(results.boxes) > 0:

        detection_count += 1

        box = results.boxes[0]

        x1, y1, x2, y2 = map(
            int,
            box.xyxy[0]
        )

        confidence = float(box.conf[0])


        # ====================================================
        # ESTIMATED TARGET
        # ====================================================

        target_x = (x1 + x2) // 2

        target_y = y1 + int(
            (y2 - y1) * TARGET_RATIO
        )


        # ====================================================
        # SMOOTH TARGET
        # ====================================================

        if smoothed_x is None:

            smoothed_x = target_x
            smoothed_y = target_y

        else:

            smoothed_x = (
                SMOOTHING * target_x
                +
                (1 - SMOOTHING) * smoothed_x
            )

            smoothed_y = (
                SMOOTHING * target_y
                +
                (1 - SMOOTHING) * smoothed_y
            )


        smoothed_x = int(smoothed_x)
        smoothed_y = int(smoothed_y)


        # ====================================================
        # PIXEL ERROR
        # ====================================================

        error_x = smoothed_x - center_x
        error_y = smoothed_y - center_y


        # ====================================================
        # BODY-FRAME VELOCITY
        # ====================================================

        #
        # X ERROR:
        #
        # target right  → positive right velocity
        # target left   → negative right velocity
        #

        velocity_right = (
            KP_RIGHT * error_x
        )


        #
        # Y ERROR:
        #
        # target above center → move forward
        # target below center → move backward
        #

        velocity_forward = (
            -KP_FORWARD * error_y
        )


        # ====================================================
        # DEAD BAND
        # ====================================================

        if abs(error_x) <= DEADBAND_X:

            velocity_right = 0


        if abs(error_y) <= DEADBAND_Y:

            velocity_forward = 0


        # ====================================================
        # LIMIT VELOCITIES
        # ====================================================

        velocity_right = max(
            -MAX_VELOCITY,
            min(MAX_VELOCITY, velocity_right)
        )

        velocity_forward = max(
            -MAX_VELOCITY,
            min(MAX_VELOCITY, velocity_forward)
        )


        # ====================================================
        # COMMAND
        # ====================================================

        commands = []


        if velocity_right > 0:

            commands.append("RIGHT")

        elif velocity_right < 0:

            commands.append("LEFT")


        if velocity_forward > 0:

            commands.append("FORWARD")

        elif velocity_forward < 0:

            commands.append("BACKWARD")


        if not commands:

            command = "HOLD POSITION"

        else:

            command = " + ".join(commands)


        # ====================================================
        # SEND COMMAND
        # ====================================================

        send_body_velocity(

            velocity_forward,

            velocity_right,

            0
        )


        # ====================================================
        # DRAW PERSON
        # ====================================================

        cv2.rectangle(

            frame,

            (x1, y1),

            (x2, y2),

            (0, 255, 0),

            2
        )


        # ====================================================
        # TARGET
        # ====================================================

        cv2.circle(

            frame,

            (target_x, target_y),

            6,

            (0, 255, 255),

            -1
        )


        cv2.circle(

            frame,

            (smoothed_x, smoothed_y),

            10,

            (0, 0, 255),

            -1
        )


        # ====================================================
        # ERROR VECTOR
        # ====================================================

        cv2.arrowedLine(

            frame,

            (center_x, center_y),

            (smoothed_x, smoothed_y),

            (0, 255, 255),

            2
        )


        # ====================================================
        # DISPLAY DATA
        # ====================================================

        cv2.putText(

            frame,

            f"PERSON {confidence:.2f}",

            (20, 35),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.65,

            (0, 255, 0),

            2
        )


        cv2.putText(

            frame,

            f"ERROR X: {error_x}px",

            (20, 70),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.6,

            (255, 255, 255),

            2
        )


        cv2.putText(

            frame,

            f"ERROR Y: {error_y}px",

            (20, 105),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.6,

            (255, 255, 255),

            2
        )


        cv2.putText(

            frame,

            f"FORWARD: {velocity_forward:.2f} m/s",

            (20, 140),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.55,

            (255, 255, 255),

            2
        )


        cv2.putText(

            frame,

            f"RIGHT: {velocity_right:.2f} m/s",

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


    # ========================================================
    # NO PERSON
    # ========================================================

    else:

        # Target lost → stop movement

        send_body_velocity(
            0,
            0,
            0
        )


        cv2.putText(

            frame,

            "NO PERSON DETECTED",

            (20, 40),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.75,

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


    # ========================================================
    # FPS
    # ========================================================

    elapsed = time.time() - start_time

    fps = (
        frame_count / elapsed
        if elapsed > 0
        else 0
    )


    cv2.putText(

        frame,

        f"FPS: {fps:.1f}",

        (width - 120, 35),

        cv2.FONT_HERSHEY_SIMPLEX,

        0.6,

        (255, 255, 255),

        2
    )


    # ========================================================
    # DISPLAY
    # ========================================================

    cv2.imshow(

        "DOXI - Body Frame Visual Servoing",

        frame
    )


    if cv2.waitKey(1) & 0xFF == ord("q"):

        break


# ============================================================
# CLEANUP
# ============================================================

send_body_velocity(0, 0, 0)

cap.release()

cv2.destroyAllWindows()

master.close()


print()
print("=" * 65)
print("VISUAL SERVOING STOPPED")
print("=" * 65)

print(f"Frames processed : {frame_count}")
print(f"Detections       : {detection_count}")

if frame_count > 0:

    print(
        f"Detection rate   : "
        f"{(detection_count / frame_count) * 100:.2f}%"
    )