from ultralytics import YOLO
model = YOLO("yolov8n.pt")  # 또는 'yolov8n.pt', 'yolov8s.yaml' 등
model.train(data='/home/rokey/rokey_ws/src/rokey_pjt/rokey_pjt/yolo_detect/data.yaml', epochs=50, imgsz=320, batch=16)