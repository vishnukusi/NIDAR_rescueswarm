import cv2
import csv
import time
from ultralytics import YOLO
from pymavlink import mavutil


# ============================================================
# DOXI CLOSED-LOOP VISUAL SERVOING EXPERIMENT
#
# Webcam
#   ↓
# YOLOv8n
#   ↓
# Person detection
#   ↓
# Target estimation
#   ↓
# Pixel error
#   ↓
# Proportional controller
#   ↓
# MAVLink body-frame velocity
#   ↓
# ArduCopter SITL
#   ↓
# Telemetry
#   ↓
# CSV benchmark
#
# NOTE:
# YOLOv8n detects PERSON, not HEAD.
# The target is currently estimated from the upper
# portion of the person bounding box.
# ============================================================


# ============================================================
# SETTINGS
# ============================================================

MODEL_PATH = "yolov8n.pt"

CAMERA_INDEX = 0

MAVLINK_CONNECTION = "udp:127.0.0.1:14550"

CONFIDENCE_THRESHOLD = 0.30

DEADBAND_X = 30
DEADBAND_Y = 30

TARGET_RATIO = 0.12

KP_FORWARD = 0.008
KP_RIGHT = 0.008

MAX_VELOCITY = 1.0

SMOOTHING = 0.25

CSV_FILE = "closed_loop_log.csv"

LOOP_DELAY = 0.01


# ============================================================
# MAVLINK CONNECTION
# ============================================================

print()
print("=" * 70)
print("DOXI CLOSED-LOOP VISUAL SERVOING")
print("=" * 70)

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
# LOAD YOLO
# ============================================================

print()
print("Loading YOLOv8n...")

model = YOLO(MODEL_PATH)

print("YOLOv8n loaded successfully.")


# ============================================================
# OPEN CAMERA
# ============================================================

print()
print("Opening laptop camera...")

cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():

    print("ERROR: Could not open camera.")

    master.close()

    raise SystemExit

print("Camera opened successfully.")
print("Press Q to stop the experiment.")


# ============================================================
# CSV
# ============================================================

csv_file = open(
    CSV_FILE,
    "w",
    newline=""
)

writer = csv.writer(csv_file)

writer.writerow([
    "time",
    "target_x",
    "target_y",
    "camera_center_x",
    "camera_center_y",
    "error_x",
    "error_y",
    "forward_cmd",
    "right_cmd",
    "drone_latitude",
    "drone_longitude",
    "drone_altitude",
    "relative_altitude",
    "drone_vx",
    "drone_vy",
    "drone_vz",
    "groundspeed",
    "heading",
    "confidence",
    "command"
])


# ============================================================
# VARIABLES
# ============================================================

smoothed_x = None
smoothed_y = None

frame_count = 0

detection_count = 0

centered_count = 0

lost_count = 0

start_time = time.time()

last_telemetry = {
    "latitude": "",
    "longitude": "",
    "altitude": "",
    "relative_altitude": "",
    "vx": "",
    "vy": "",
    "vz": "",
    "groundspeed": "",
    "heading": ""
}


# ============================================================
# SEND BODY VELOCITY
# ============================================================

def send_body_velocity(
    forward,
    right,
    down
):

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
# READ TELEMETRY
# ============================================================

def read_telemetry():

    global last_telemetry

    # --------------------------------------------------------
    # GLOBAL POSITION
    # --------------------------------------------------------

    position = master.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=False
    )

    if position is not None:

        last_telemetry["latitude"] = (
            position.lat / 1e7
        )

        last_telemetry["longitude"] = (
            position.lon / 1e7
        )

        last_telemetry["altitude"] = (
            position.alt / 1000.0
        )

        last_telemetry["relative_altitude"] = (
            position.relative_alt / 1000.0
        )

        last_telemetry["vx"] = (
            position.vx / 100.0
        )

        last_telemetry["vy"] = (
            position.vy / 100.0
        )

        last_telemetry["vz"] = (
            position.vz / 100.0
        )

        last_telemetry["heading"] = (
            position.hdg / 100.0
        )


    # --------------------------------------------------------
    # VFR HUD
    # --------------------------------------------------------

    hud = master.recv_match(
        type="VFR_HUD",
        blocking=False
    )

    if hud is not None:

        last_telemetry["groundspeed"] = (
            hud.groundspeed
        )

        last_telemetry["heading"] = (
            hud.heading
        )


# ============================================================
# MAIN LOOP
# ============================================================

try:

    while True:

        ret, frame = cap.read()

        if not ret:

            print("ERROR: Camera frame unavailable.")

            break


        frame_count += 1


        # ====================================================
        # IMAGE SIZE
        # ====================================================

        height, width = frame.shape[:2]

        center_x = width // 2

        center_y = height // 2


        # ====================================================
        # READ TELEMETRY
        # ====================================================

        read_telemetry()


        # ====================================================
        # YOLO
        # ====================================================

        results = model(

            frame,

            imgsz=640,

            conf=CONFIDENCE_THRESHOLD,

            classes=[0],

            max_det=1,

            verbose=False

        )[0]


        # ====================================================
        # DRAW CAMERA CENTER
        # ====================================================

        cv2.circle(

            frame,

            (center_x, center_y),

            8,

            (255, 0, 0),

            -1
        )


        # ====================================================
        # DEFAULT CONTROL VALUES
        # ====================================================

        forward_cmd = 0.0

        right_cmd = 0.0

        command = "HOLD"


        target_x = ""

        target_y = ""

        error_x = ""

        error_y = ""

        confidence = ""


        # ====================================================
        # PERSON DETECTED
        # ====================================================

        if len(results.boxes) > 0:

            detection_count += 1

            lost_count = 0

            box = results.boxes[0]

            x1, y1, x2, y2 = map(
                int,
                box.xyxy[0]
            )

            confidence = float(
                box.conf[0]
            )


            # ------------------------------------------------
            # ESTIMATED TARGET
            # ------------------------------------------------

            raw_target_x = (
                x1 + x2
            ) // 2

            raw_target_y = (
                y1
                +
                int(
                    (y2 - y1)
                    * TARGET_RATIO
                )
            )


            # ------------------------------------------------
            # SMOOTH TARGET
            # ------------------------------------------------

            if smoothed_x is None:

                smoothed_x = raw_target_x

                smoothed_y = raw_target_y

            else:

                smoothed_x = (

                    SMOOTHING
                    * raw_target_x

                    +

                    (1 - SMOOTHING)
                    * smoothed_x

                )

                smoothed_y = (

                    SMOOTHING
                    * raw_target_y

                    +

                    (1 - SMOOTHING)
                    * smoothed_y

                )


            smoothed_x = int(
                smoothed_x
            )

            smoothed_y = int(
                smoothed_y
            )


            target_x = smoothed_x

            target_y = smoothed_y


            # ------------------------------------------------
            # PIXEL ERROR
            # ------------------------------------------------

            error_x = (
                target_x
                -
                center_x
            )

            error_y = (
                target_y
                -
                center_y
            )


            # ------------------------------------------------
            # PROPORTIONAL CONTROL
            # ------------------------------------------------

            right_cmd = (
                KP_RIGHT
                *
                error_x
            )

            forward_cmd = (
                -KP_FORWARD
                *
                error_y
            )


            # ------------------------------------------------
            # DEAD BAND
            # ------------------------------------------------

            if abs(error_x) <= DEADBAND_X:

                right_cmd = 0.0


            if abs(error_y) <= DEADBAND_Y:

                forward_cmd = 0.0


            # ------------------------------------------------
            # VELOCITY LIMIT
            # ------------------------------------------------

            right_cmd = max(
                -MAX_VELOCITY,
                min(
                    MAX_VELOCITY,
                    right_cmd
                )
            )

            forward_cmd = max(
                -MAX_VELOCITY,
                min(
                    MAX_VELOCITY,
                    forward_cmd
                )
            )


            # ------------------------------------------------
            # COMMAND DESCRIPTION
            # ------------------------------------------------

            commands = []


            if right_cmd > 0:

                commands.append("RIGHT")

            elif right_cmd < 0:

                commands.append("LEFT")


            if forward_cmd > 0:

                commands.append("FORWARD")

            elif forward_cmd < 0:

                commands.append("BACKWARD")


            if not commands:

                command = "HOLD"

                centered_count += 1

            else:

                command = " + ".join(
                    commands
                )


            # ------------------------------------------------
            # SEND MAVLINK
            # ------------------------------------------------

            send_body_velocity(

                forward_cmd,

                right_cmd,

                0
            )


            # ------------------------------------------------
            # DRAW PERSON
            # ------------------------------------------------

            cv2.rectangle(

                frame,

                (x1, y1),

                (x2, y2),

                (0, 255, 0),

                2
            )


            # ------------------------------------------------
            # RAW TARGET
            # ------------------------------------------------

            cv2.circle(

                frame,

                (raw_target_x, raw_target_y),

                5,

                (0, 255, 255),

                -1
            )


            # ------------------------------------------------
            # SMOOTH TARGET
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

                (center_x, center_y),

                (target_x, target_y),

                (0, 255, 255),

                2
            )


            # ------------------------------------------------
            # TEXT
            # ------------------------------------------------

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

                f"TARGET: ({target_x}, {target_y})",

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

                f"FORWARD: {forward_cmd:.2f} m/s",

                (20, 155),

                cv2.FONT_HERSHEY_SIMPLEX,

                0.55,

                (255, 255, 255),

                2
            )


            cv2.putText(

                frame,

                f"RIGHT: {right_cmd:.2f} m/s",

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


        # ====================================================
        # NO PERSON
        # ====================================================

        else:

            lost_count += 1

            send_body_velocity(
                0,
                0,
                0
            )

            command = "SEARCH / HOLD"

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

                "MAVLINK: ZERO VELOCITY",

                (20, 80),

                cv2.FONT_HERSHEY_SIMPLEX,

                0.60,

                (0, 0, 255),

                2
            )


        # ====================================================
        # LOG DATA
        # ====================================================

        elapsed = (
            time.time()
            -
            start_time
        )


        writer.writerow([

            f"{elapsed:.3f}",

            target_x,

            target_y,

            center_x,

            center_y,

            error_x,

            error_y,

            f"{forward_cmd:.4f}",

            f"{right_cmd:.4f}",

            last_telemetry["latitude"],

            last_telemetry["longitude"],

            last_telemetry["altitude"],

            last_telemetry["relative_altitude"],

            last_telemetry["vx"],

            last_telemetry["vy"],

            last_telemetry["vz"],

            last_telemetry["groundspeed"],

            last_telemetry["heading"],

            confidence,

            command

        ])


        csv_file.flush()


        # ====================================================
        # FPS
        # ====================================================

        elapsed_total = (
            time.time()
            -
            start_time
        )

        fps = (

            frame_count
            /
            elapsed_total

            if elapsed_total > 0

            else 0
        )


        cv2.putText(

            frame,

            f"FPS: {fps:.1f}",

            (width - 120, 35),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.60,

            (255, 255, 255),

            2
        )


        # ====================================================
        # DISPLAY TELEMETRY
        # ====================================================

        altitude = (
            last_telemetry["relative_altitude"]
        )

        groundspeed = (
            last_telemetry["groundspeed"]
        )


        cv2.putText(

            frame,

            f"ALT: {altitude}",

            (width - 180, 65),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.50,

            (255, 255, 255),

            2
        )


        cv2.putText(

            frame,

            f"GS: {groundspeed}",

            (width - 180, 95),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.50,

            (255, 255, 255),

            2
        )


        # ====================================================
        # DISPLAY
        # ====================================================

        cv2.imshow(

            "DOXI Closed-Loop Visual Servoing",

            frame
        )


        if cv2.waitKey(1) & 0xFF == ord("q"):

            break


        time.sleep(
            LOOP_DELAY
        )


# ============================================================
# STOP SAFELY
# ============================================================

except KeyboardInterrupt:

    print()
    print("Experiment interrupted.")


finally:

    send_body_velocity(
        0,
        0,
        0
    )

    cap.release()

    cv2.destroyAllWindows()

    csv_file.close()

    master.close()


# ============================================================
# RESULTS
# ============================================================

print()
print("=" * 70)
print("CLOSED-LOOP EXPERIMENT FINISHED")
print("=" * 70)

print(
    f"Frames processed : {frame_count}"
)

print(
    f"Detections       : {detection_count}"
)

print(
    f"Centered frames  : {centered_count}"
)

print(
    f"CSV saved        : {CSV_FILE}"
)

if frame_count > 0:

    print(
        f"Detection rate   : "
        f"{(detection_count / frame_count) * 100:.2f}%"
    )

print()
print("Experiment complete.")