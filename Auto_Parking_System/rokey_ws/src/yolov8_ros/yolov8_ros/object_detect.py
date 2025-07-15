import cv2
import random
import os
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO# 기본 카메라: 0, realsense gray: 4, realsense: 6
CAMERA_NUM = 0# 신뢰도
CONFIDENCE = 0.40# YOLO 모델 로드
#package_share_directory = get_package_share_directory('rokey_project')
weights = '/home/seoyoon/Disabled_best.pt'
model = YOLO(weights)# 클래스별 고유 색상 생성 (랜덤 색상 생성)
class_names = model.names  # 딕셔너리 형태: {0: 'class0', 1: 'class1', ...}
class_colors = {cls_id: (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
                for cls_id in class_names}# 웹캠 열기
cap = cv2.VideoCapture(CAMERA_NUM)
if not cap.isOpened():
    print(":x: 웹캠을 열 수 없습니다.")
    exit()
    while True:
        ret, frame = cap.read()
        if not ret:
            print(":x: 프레임을 읽을 수 없습니다.")
            break    # YOLO 추론
        results = model(frame)    # 일정 신뢰도 이상인 박스만 필터링
        filtered_boxes = []
        for box in results[0].boxes:
            if box.conf.item() >= CONFIDENCE:
                filtered_boxes.append(box)    # 시각화
        annotated_frame = frame.copy()
        for box in filtered_boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = box.conf.item()
            cls = int(box.cls[0])
            # label = f"{class_names[cls]} {conf:.2f}"  # 클래스 이름, 신뢰도 출력
            label = f"{conf:.2f}"   # 신뢰도 출력
            color = class_colors.get(cls, (0, 255, 0))  # 기본색: 초록        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)    # 화면 출력
    cv2.imshow(f"YOLOv8 Webcam Detection (Conf >= {CONFIDENCE})", annotated_frame)    # 'q' 키로 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()
