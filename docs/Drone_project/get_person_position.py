import cv2
from ultralytics import YOLO

model = YOLO("yolov8n.pt")

img = cv2.imread("person_images/person1.jpg")

if img is None:
    print("Image not found!")
    exit()

# Get the first detected person
results = model(img, classes=[0], conf=0.3)[0]

if len(results.boxes) == 0:
    print("No person detected")
else:
    box = results.boxes[0]

    x1, y1, x2, y2 = map(int, box.xyxy[0])

    # Person center
    person_x = (x1 + x2) // 2
    person_y = (y1 + y2) // 2

    # Image center
    height, width = img.shape[:2]
    image_x = width // 2
    image_y = height // 2

    print("Person center:", person_x, person_y)
    print("Image center:", image_x, image_y)

    # Error
    error_x = person_x - image_x
    error_y = person_y - image_y

    print("Horizontal error:", error_x)
    print("Vertical error:", error_y)

    # Visualize centers
    cv2.circle(img, (person_x, person_y), 8, (0, 0, 255), -1)
    cv2.circle(img, (image_x, image_y), 8, (255, 0, 0), -1)

    cv2.imshow("Position Analysis", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()