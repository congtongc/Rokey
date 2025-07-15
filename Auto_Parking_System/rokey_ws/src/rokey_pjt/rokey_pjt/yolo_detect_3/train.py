from ultralytics import YOLO
model = YOLO("yolo11s.pt")  # 또는 'yolov8n.pt', 'yolov8s.yaml' 등
model.train(data='/home/rokey/rokey_ws/src/rokey_pjt/rokey_pjt/yolo_detect_3/data.yaml', epochs=50, imgsz=640, batch=16)