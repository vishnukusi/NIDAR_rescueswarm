import cv2
from ultralytics import YOLO

print("Loading model...")
model = YOLO("yolov8n.pt")

print("Loading image...")
img = cv2.imread("person_images/person1.jpg")

if img is None:
    print("ERROR: Image not found!")
    exit()

print("Original image shape:", img.shape)

print("\nRunning YOLO detection...")

results = model(
    img,
    imgsz=1280,
    conf=0.05,
    classes=[0],
    verbose=False
)[0]

print("\nNumber of persons detected:", len(results.boxes))

output = img.copy()

for i, box in enumerate(results.boxes):
    x1, y1, x2, y2 = map(int, box.xyxy[0])
    confidence = float(box.conf[0])

    print(f"\nPerson {i + 1}")
    print(f"Bounding box: ({x1}, {y1}) to ({x2}, {y2})")
    print(f"Confidence: {confidence:.3f}")

    cv2.rectangle(
        output,
        (x1, y1),
        (x2, y2),
        (0, 255, 0),
        2
    )

    cv2.putText(
        output,
        f"Person {confidence:.2f}",
        (x1, y1 - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2
    )

cv2.imwrite("test_detection_result.jpg", output)

print("\nResult saved as: test_detection_result.jpg")

cv2.imshow("YOLO Detection Test", output)
cv2.waitKey(0)
cv2.destroyAllWindows()