import cv2
import numpy as np
import math
import json
import os
import time
import glob
from ultralytics import YOLO


# ============================================================
# SETTINGS
# ============================================================

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720

CAMERA_CENTER_X = IMAGE_WIDTH // 2
CAMERA_CENTER_Y = IMAGE_HEIGHT // 2

CENTER_THRESHOLD = 30
MOVEMENT_STEP_PIXELS = 30
MAX_SIMULATION_STEPS = 20

# Head is estimated near the upper part of the person box
HEAD_POSITION_RATIO = 0.12


# ============================================================
# PIXEL -> GPS
# ============================================================

def get_gps_from_pixel_downward(lat, lon, h, u, v, fx, fy, cx, cy, yaw):

    dx_pix = u - cx
    dy_pix = v - cy

    E = -(dx_pix / fx) * h
    N = -(dy_pix / fy) * h

    distance = math.hypot(E, N)

    rel_bearing = (
        math.degrees(math.atan2(E, N)) + 360
    ) % 360

    yaw = (360 - yaw) % 360
    true_bearing = (rel_bearing + yaw) % 360

    R_EARTH = 6378137.0

    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    br = math.radians(true_bearing)

    lat2 = math.asin(
        math.sin(lat1) * math.cos(distance / R_EARTH)
        + math.cos(lat1)
        * math.sin(distance / R_EARTH)
        * math.cos(br)
    )

    lon2 = lon1 + math.atan2(
        math.sin(br)
        * math.sin(distance / R_EARTH)
        * math.cos(lat1),

        math.cos(distance / R_EARTH)
        - math.sin(lat1) * math.sin(lat2)
    )

    return (
        math.degrees(lat2),
        math.degrees(lon2),
        distance,
        rel_bearing,
        true_bearing
    )


# ============================================================
# DETECT PERSON AND ESTIMATE HEAD POSITION
# ============================================================

def detect_person_and_head(img, model):

    results = model(
        img,
        imgsz=768,
        conf=0.15,
        iou=0.4,
        classes=[0],
        max_det=1,
        verbose=False
    )[0]

    if len(results.boxes) == 0:
        return None

    box = results.boxes[0]

    x1, y1, x2, y2 = map(int, box.xyxy[0])
    confidence = float(box.conf[0])

    # Person center - only for display
    person_center_x = (x1 + x2) // 2
    person_center_y = (y1 + y2) // 2

    # Estimated head target position
    head_x = (x1 + x2) // 2
    head_y = y1 + int((y2 - y1) * HEAD_POSITION_RATIO)

    return {
        "x1": x1,
        "y1": y1,
        "x2": x2,
        "y2": y2,

        "person_center_x": person_center_x,
        "person_center_y": person_center_y,

        "head_x": head_x,
        "head_y": head_y,

        "confidence": confidence
    }


# ============================================================
# DECIDE MOVEMENT
# ============================================================

def get_movement_command(error_x, error_y):

    commands = []

    if error_x > CENTER_THRESHOLD:
        commands.append("RIGHT")

    elif error_x < -CENTER_THRESHOLD:
        commands.append("LEFT")

    if error_y > CENTER_THRESHOLD:
        commands.append("DOWN")

    elif error_y < -CENTER_THRESHOLD:
        commands.append("UP")

    if not commands:
        return "HOLD"

    return " + ".join(commands)


# ============================================================
# MOVE VIRTUAL CAMERA
# ============================================================

def move_virtual_camera(img, command):

    shift_x = 0
    shift_y = 0

    # Horizontal movement
    if "RIGHT" in command:
        shift_x = -MOVEMENT_STEP_PIXELS

    elif "LEFT" in command:
        shift_x = MOVEMENT_STEP_PIXELS

    # Vertical movement
    if "DOWN" in command:
        shift_y = -MOVEMENT_STEP_PIXELS

    elif "UP" in command:
        shift_y = MOVEMENT_STEP_PIXELS

    M = np.float32([
        [1, 0, shift_x],
        [0, 1, shift_y]
    ])

    shifted_img = cv2.warpAffine(
        img,
        M,
        (IMAGE_WIDTH, IMAGE_HEIGHT)
    )

    return shifted_img


# ============================================================
# DRAW SIMULATION
# ============================================================

def draw_simulation(img, detection, step, command):

    output = img.copy()

    # --------------------------------------------------------
    # Camera center - BLUE
    # --------------------------------------------------------

    cv2.circle(
        output,
        (CAMERA_CENTER_X, CAMERA_CENTER_Y),
        9,
        (255, 0, 0),
        -1
    )

    cv2.putText(
        output,
        "CAMERA CENTER",
        (CAMERA_CENTER_X + 15, CAMERA_CENTER_Y - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 0, 0),
        2
    )

    if detection is None:
        return output

    x1 = detection["x1"]
    y1 = detection["y1"]
    x2 = detection["x2"]
    y2 = detection["y2"]

    head_x = detection["head_x"]
    head_y = detection["head_y"]

    # --------------------------------------------------------
    # Person bounding box - GREEN
    # --------------------------------------------------------

    cv2.rectangle(
        output,
        (x1, y1),
        (x2, y2),
        (0, 255, 0),
        2
    )

    # --------------------------------------------------------
    # Estimated head target - RED
    # --------------------------------------------------------

    cv2.circle(
        output,
        (head_x, head_y),
        10,
        (0, 0, 255),
        -1
    )

    cv2.putText(
        output,
        "HEAD TARGET",
        (head_x + 15, head_y - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 0, 255),
        2
    )

    # --------------------------------------------------------
    # Error line
    # --------------------------------------------------------

    cv2.line(
        output,
        (CAMERA_CENTER_X, CAMERA_CENTER_Y),
        (head_x, head_y),
        (0, 255, 255),
        2
    )

    # --------------------------------------------------------
    # Calculate errors
    # --------------------------------------------------------

    error_x = head_x - CAMERA_CENTER_X
    error_y = head_y - CAMERA_CENTER_Y

    # --------------------------------------------------------
    # Display information
    # --------------------------------------------------------

    cv2.putText(
        output,
        f"STEP: {step}",
        (30, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2
    )

    cv2.putText(
        output,
        f"ERROR X: {error_x}",
        (30, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2
    )

    cv2.putText(
        output,
        f"ERROR Y: {error_y}",
        (30, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2
    )

    cv2.putText(
        output,
        f"COMMAND: {command}",
        (30, 160),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2
    )

    return output


# ============================================================
# VISUAL CLOSED-LOOP SIMULATION
# ============================================================

def simulate_drone_positioning(original_img, model):

    current_img = original_img.copy()

    print("\n================================================")
    print("STARTING HEAD-TARGET CLOSED-LOOP SIMULATION")
    print("================================================")

    for step in range(1, MAX_SIMULATION_STEPS + 1):

        # Detect person and estimate head
        detection = detect_person_and_head(
            current_img,
            model
        )

        if detection is None:

            print(f"\nSTEP {step}")
            print("NO PERSON DETECTED")
            print("COMMAND: SEARCH")

            return False, current_img, None

        # Head target position
        head_x = detection["head_x"]
        head_y = detection["head_y"]

        # Error between head and camera center
        error_x = head_x - CAMERA_CENTER_X
        error_y = head_y - CAMERA_CENTER_Y

        # Decide movement
        command = get_movement_command(
            error_x,
            error_y
        )

        print(f"\nSTEP {step}")

        print(
            f"Head target: ({head_x}, {head_y})"
        )

        print(
            f"Camera center: "
            f"({CAMERA_CENTER_X}, {CAMERA_CENTER_Y})"
        )

        print(f"Error X: {error_x} pixels")
        print(f"Error Y: {error_y} pixels")
        print(f"Command: {command}")

        # Draw current state
        display_img = draw_simulation(
            current_img,
            detection,
            step,
            command
        )

        cv2.imshow(
            "DOXI Drone Positioning Simulation",
            display_img
        )

        cv2.waitKey(700)

        # Target centered
        if command == "HOLD":

            print("\nHEAD TARGET CENTERED!")
            print("FINAL COMMAND: HOLD POSITION")

            return True, current_img, detection

        # Move virtual camera
        current_img = move_virtual_camera(
            current_img,
            command
        )

    print("\nMAXIMUM STEPS REACHED")
    print("HEAD TARGET NOT CENTERED")

    return False, current_img, None


# ============================================================
# PROCESS IMAGE
# ============================================================

def process_image_json(
    image_path,
    json_path,
    model,
    map1,
    map2,
    K
):

    with open(json_path, "r") as f:
        cfg = json.load(f)

    drone = cfg["drone"]

    LAT_DRONE = drone["latitude"]
    LON_DRONE = drone["longitude"]
    ALTITUDE = drone["altitude_m"]
    YAW = drone.get("yaw", 0.0)

    print("\n================================================")
    print("PROCESSING IMAGE:", image_path)
    print("================================================")

    print(f"Drone Latitude : {LAT_DRONE}")
    print(f"Drone Longitude: {LON_DRONE}")
    print(f"Drone Altitude : {ALTITUDE} m")
    print(f"Drone Yaw      : {YAW} degrees")

    # Load image
    img = cv2.imread(image_path)

    if img is None:
        print("ERROR: IMAGE NOT FOUND")
        return

    img = cv2.resize(
        img,
        (IMAGE_WIDTH, IMAGE_HEIGHT)
    )

    # Undistort image
    img = cv2.remap(
        img,
        map1,
        map2,
        cv2.INTER_LINEAR
    )

    # Run simulation
    centered, final_img, final_detection = (
        simulate_drone_positioning(
            img,
            model
        )
    )

    # Final result
    if centered and final_detection is not None:

        head_x = final_detection["head_x"]
        head_y = final_detection["head_y"]

        (
            p_lat,
            p_lon,
            distance,
            rel_bearing,
            true_bearing
        ) = get_gps_from_pixel_downward(
            LAT_DRONE,
            LON_DRONE,
            ALTITUDE,
            head_x,
            head_y,
            K[0, 0],
            K[1, 1],
            K[0, 2],
            K[1, 2],
            YAW
        )

        print("\n================================================")
        print("FINAL RESULT")
        print("================================================")

        print("STATUS: HEAD TARGET CENTERED")

        print(
            f"Final Head Position: "
            f"({head_x}, {head_y})"
        )

        print(f"Estimated Latitude: {p_lat}")
        print(f"Estimated Longitude: {p_lon}")
        print(f"Final Distance: {distance:.2f} m")
        print("FINAL COMMAND: HOLD POSITION")

        # Save result to log
        with open(
            "drone_logs.csv",
            "a",
            newline=""
        ) as f:

            f.write(
                f"HEAD TARGET CENTERED,"
                f"Drone 1,"
                f"{p_lat},"
                f"{p_lon},"
                f"{ALTITUDE},"
                f"{time.strftime('%Y-%m-%d %H:%M:%S')}\n"
            )

    else:

        print("\nSIMULATION FINISHED WITHOUT CENTERING HEAD")

    # Save final image
    output_file = "final_head_target_result.jpg"

    cv2.imwrite(
        output_file,
        final_img
    )

    print(
        f"\nFinal image saved: {output_file}"
    )

    cv2.imshow(
        "Final Head Target Result",
        final_img
    )

    cv2.waitKey(0)

    cv2.destroyAllWindows()


# ============================================================
# MAIN
# ============================================================

def main():

    print("Program started")
    print("Current folder:", os.getcwd())

    # Load YOLO model
    print("\nLoading YOLOv8n model...")

    model = YOLO("yolov8n.pt")

    print("Model loaded successfully!")

    # --------------------------------------------------------
    # Camera calibration
    # --------------------------------------------------------

    K = np.array([
        [1466.75124, 0, 434.400604],
        [0, 1443.50616, 362.768062],
        [0, 0, 1]
    ])

    dist = np.array([
        -0.690061703,
        3.10678069,
        0.00428299,
        0.0273968,
        -9.87146169
    ])

    new_K, roi = cv2.getOptimalNewCameraMatrix(
        K,
        dist,
        (IMAGE_WIDTH, IMAGE_HEIGHT),
        alpha=0
    )

    map1, map2 = cv2.initUndistortRectifyMap(
        K,
        dist,
        None,
        new_K,
        (IMAGE_WIDTH, IMAGE_HEIGHT),
        cv2.CV_16SC2
    )

    # Find images
    image_files = sorted(
        glob.glob("person_images/person*.jpg")
    )

    print("Images found:", image_files)

    pairs = []

    for img_path in image_files:

        base = os.path.splitext(
            os.path.basename(img_path)
        )[0]

        json_path = f"person_images/{base}.json"

        if os.path.exists(json_path):
            pairs.append((img_path, json_path))

    print("Image/JSON pairs:", pairs)

    if not pairs:

        print("\nERROR: NO VALID IMAGE/JSON PAIRS")
        return

    # Process every image
    for image_path, json_path in pairs:

        process_image_json(
            image_path,
            json_path,
            model,
            map1,
            map2,
            K
        )


# ============================================================
# START
# ============================================================

if __name__ == "__main__":
    main()