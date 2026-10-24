from ultralytics import YOLO

model = YOLO("yolov8n.pt")

print("Model loaded successfully!")
print("Classes:")
print(model.names)