import cv2
from ultralytics import YOLO
import time


# ============================================================
# DOXI VISUAL SERVOING CONTROLLER
# ============================================================

MODEL_PATH = "yolov8n.pt"
CAMERA_INDEX = 0

CONFIDENCE_THRESHOLD = 0.30

# ------------------------------------------------------------
# Camera center deadband
# If target is inside this region -> HOLD
# ------------------------------------------------------------

DEADBAND_X = 30
DEADBAND_Y = 30

# ------------------------------------------------------------
# Controller
# ------------------------------------------------------------

KP_X = 0.008
KP_Y = 0.008

MAX_VELOCITY = 1.0

# ------------------------------------------------------------
# Smoothing
# 0 = completely old value
# 1 = completely new value
# ------------------------------------------------------------

SMOOTHING = 0.25

# ------------------------------------------------------------
# Target estimation
#
# YOLOv8n detects PERSON.
# It does not detect HEAD.
#
# We estimate a target point near the upper part
# of the person bounding box.
# ------------------------------------------------------------

TARGET_RATIO = 0.12

# ------------------------------------------------------------
# Detection loss tolerance
# Number of consecutive frames allowed without detection
# ------------------------------------------------------------

MAX_LOST_FRAMES = 10


# ============================================================
# LOAD MODEL
# ============================================================

print()
print("=" * 65)
print("DOXI VISUAL SERVOING CONTROLLER")
print("=" * 65)

print()
print("Loading YOLOv8n...")

model = YOLO(MODEL_PATH)

print("Model loaded successfully.")


# ============================================================
# OPEN CAMERA
# ============================================================

print()
print("Opening camera...")

cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():

    print("ERROR: Camera could not be opened.")
    exit()

print("Camera started.")
print("Press Q to quit.")


# ============================================================
# VARIABLES
# ============================================================

smoothed_x = None
smoothed_y = None

lost_frames = 0

frame_count = 0
detection_count = 0
centered_count = 0

start_time = time.time()


# ============================================================
# MAIN LOOP
# ============================================================

while True:

    ret, frame = cap.read()

    if not ret:

        print("ERROR: Could not read camera frame.")
        break


    frame_count += 1

    height, width = frame.shape[:2]

    camera_center_x = width // 2
    camera_center_y = height // 2


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
        (camera_center_x, camera_center_y),
        8,
        (255, 0, 0),
        -1
    )

    cv2.line(
        frame,
        (camera_center_x - 25, camera_center_y),
        (camera_center_x + 25, camera_center_y),
        (255, 0, 0),
        2
    )

    cv2.line(
        frame,
        (camera_center_x, camera_center_y - 25),
        (camera_center_x, camera_center_y + 25),
        (255, 0, 0),
        2
    )


    # ========================================================
    # PERSON DETECTED
    # ========================================================

    if len(results.boxes) > 0:

        detection_count += 1

        lost_frames = 0

        box = results.boxes[0]

        x1, y1, x2, y2 = map(
            int,
            box.xyxy[0]
        )

        confidence = float(box.conf[0])


        # ----------------------------------------------------
        # ESTIMATE TARGET
        # ----------------------------------------------------

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

        error_x = smoothed_x - camera_center_x
        error_y = smoothed_y - camera_center_y


        # ====================================================
        # PROPORTIONAL CONTROL
        # ====================================================

        velocity_x = KP_X * error_x
        velocity_y = KP_Y * error_y


        # ====================================================
        # APPLY DEADBAND
        # ====================================================

        if abs(error_x) <= DEADBAND_X:

            velocity_x = 0

        if abs(error_y) <= DEADBAND_Y:

            velocity_y = 0


        # ====================================================
        # LIMIT VELOCITY
        # ========================================================

        velocity_x = max(
            -MAX_VELOCITY,
            min(MAX_VELOCITY, velocity_x)
        )

        velocity_y = max(
            -MAX_VELOCITY,
            min(MAX_VELOCITY, velocity_y)
        )


        # ====================================================
        # MOVEMENT COMMAND
        # ====================================================

        commands = []


        if velocity_x > 0:

            commands.append("RIGHT")

        elif velocity_x < 0:

            commands.append("LEFT")


        if velocity_y > 0:

            commands.append("DOWN")

        elif velocity_y < 0:

            commands.append("UP")


        if len(commands) == 0:

            command = "HOLD POSITION"

            centered_count += 1

        else:

            command = "MOVE " + " + ".join(commands)


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
        # DRAW RAW TARGET
        # ====================================================

        cv2.circle(
            frame,
            (target_x, target_y),
            6,
            (0, 255, 255),
            -1
        )


        # ====================================================
        # DRAW SMOOTHED TARGET
        # ====================================================

        cv2.circle(
            frame,
            (smoothed_x, smoothed_y),
            10,
            (0, 0, 255),
            -1
        )


        cv2.putText(
            frame,
            "TARGET",
            (
                smoothed_x + 12,
                smoothed_y
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            2
        )


        # ====================================================
        # ERROR VECTOR
        # ====================================================

        cv2.arrowedLine(
            frame,
            (camera_center_x, camera_center_y),
            (smoothed_x, smoothed_y),
            (0, 255, 255),
            2
        )


        # ====================================================
        # INFORMATION
        # ====================================================

        cv2.putText(
            frame,
            f"PERSON: {confidence:.2f}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            f"TARGET: ({smoothed_x}, {smoothed_y})",
            (20, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"ERROR X: {error_x}px",
            (20, 95),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"ERROR Y: {error_y}px",
            (20, 125),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"VELOCITY X: {velocity_x:.2f} m/s",
            (20, 155),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"VELOCITY Y: {velocity_y:.2f} m/s",
            (20, 185),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"COMMAND: {command}",
            (20, 220),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2
        )


    # ========================================================
    # PERSON NOT DETECTED
    # ========================================================

    else:

        lost_frames += 1


        if lost_frames <= MAX_LOST_FRAMES:

            command = "TARGET TEMPORARILY LOST"

        else:

            command = "SEARCH"


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
            f"LOST FRAMES: {lost_frames}",
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"COMMAND: {command}",
            (20, 110),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2
        )


    # ========================================================
    # FPS
    # ========================================================

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
        0.6,
        (255, 255, 255),
        2
    )


    # ========================================================
    # SHOW
    # ========================================================

    cv2.imshow(
        "DOXI - Visual Servoing Controller",
        frame
    )


    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):

        break


# ============================================================
# CLEANUP
# ============================================================

cap.release()
cv2.destroyAllWindows()


# ============================================================
# RESULTS
# ============================================================

print()
print("=" * 65)
print("DOXI VISUAL SERVOING RESULTS")
print("=" * 65)

print(f"Frames processed      : {frame_count}")
print(f"Person detections     : {detection_count}")

if frame_count > 0:

    detection_rate = (
        detection_count / frame_count
    ) * 100

    print(
        f"Detection rate        : "
        f"{detection_rate:.2f}%"
    )

if detection_count > 0:

    centered_rate = (
        centered_count / detection_count
    ) * 100

    print(
        f"Centered frames       : "
        f"{centered_rate:.2f}%"
    )

print()
print("Simulation stopped.")