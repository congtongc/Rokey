#!/usr/bin/env python3
# bridge_server.py
import json
import asyncio
import uvicorn
import threading
import time
from datetime import datetime
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from zeroconf import ServiceInfo, Zeroconf
import socket
import logging

# DB 관리자 임포트
from rokey_pjt.db_manager import DBManager

# 로거 설정
logger = logging.getLogger("bridge_server")
logger.setLevel(logging.INFO)
handler = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
if not logger.hasHandlers():
    logger.addHandler(handler)

print("=== [DEBUG] bridge_server.py 시작됨 ===")
logger.info("=== [DEBUG] bridge_server.py logger 시작됨 ===")

# 서버 앱 생성
app = FastAPI(title="Parking System Bridge Server")

# CORS 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,  # credentials가 필요없으므로 False로 설정
    allow_methods=["*"],
    allow_headers=["*"],
)

# 전역 변수
latest_camera_frame = None
latest_ocr_result = {"car_plate": "", "type": "normal"}
connected_websockets = []
db_manager = DBManager()
# camera_sub = '/robot2/oakd/rgb/image_raw'
camera_sub = '/robot3/oakd/rgb/image_raw'
ocr_sub = '/carplate/ocr_result'
location_pub = '/parking/location'

# 서버 상태 추적을 위한 전역 변수 추가
server_status = {
    "start_time": datetime.now().isoformat(),
    "connected_clients": 0,
    "last_camera_frame": None,
    "last_ocr_result": None
}

# 이벤트 루프 관리를 위한 전역 변수 추가
main_event_loop = None

# 데이터 모델
class ParkingRequest(BaseModel):
    license_plate: str
    car_type: str = "normal"

class LocationRequest(BaseModel):
    location: str

class OcrResult(BaseModel):
    car_plate: str
    type: str = "normal"

# ROS2 노드 클래스
class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')
        self.bridge = CvBridge()
        
        # 카메라 토픽 구독
        self.camera_subscription = self.create_subscription(
            Image,
            camera_sub,
            self.camera_callback,
            10)
            
        # OCR 결과 토픽 구독
        self.ocr_subscription = self.create_subscription(
            String,
            ocr_sub,
            self.ocr_callback,
            10)
            
        # 위치 발행자
        self.location_publisher = self.create_publisher(
            String,
            location_pub,
            10)
            
        self.get_logger().info('브릿지 노드 초기화 완료')
    
    def camera_callback(self, msg):
        """카메라 콜백"""
        global latest_camera_frame
        try:
            # ROS 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            latest_camera_frame = cv_image
            
            # 이벤트 루프가 있는 경우에만 브로드캐스트 실행
            if main_event_loop is not None and main_event_loop.is_running():
                main_event_loop.call_soon_threadsafe(
                    lambda: asyncio.run_coroutine_threadsafe(
                        broadcast_camera_frame(), 
                        main_event_loop
                    )
                )
        except Exception as e:
            self.get_logger().error(f'카메라 콜백 오류: {e}')
    
    def ocr_callback(self, msg):
        """OCR 결과 콜백"""
        global latest_ocr_result
        try:
            # JSON 문자열을 파싱
            data = json.loads(msg.data)
            latest_ocr_result = {
                "car_plate": data.get('car_plate', ''),
                "type": data.get('type', 'normal')
            }
            
            # 'electric' 타입을 'ev'로 변환
            if latest_ocr_result["type"] == 'electric':
                latest_ocr_result["type"] = 'ev'
                
            # 이벤트 루프가 있는 경우에만 브로드캐스트 실행
            if main_event_loop is not None and main_event_loop.is_running():
                main_event_loop.call_soon_threadsafe(
                    lambda: asyncio.run_coroutine_threadsafe(
                        broadcast_ocr_result(), 
                        main_event_loop
                    )
                )
        except Exception as e:
            self.get_logger().error(f'OCR 콜백 오류: {e}')
    
    def publish_location(self, location):
        """위치 정보 발행"""
        try:
            msg = String()
            msg.data = location
            self.location_publisher.publish(msg)
            self.get_logger().info(f'위치 정보 발행: {location}')
            return True
        except Exception as e:
            self.get_logger().error(f'위치 정보 발행 오류: {e}')
            return False

# 웹소켓 브로드캐스트 함수
async def broadcast_camera_frame():
    """모든 연결된 클라이언트에 카메라 프레임 전송"""
    if latest_camera_frame is not None and connected_websockets:
        # JPEG로 인코딩
        _, jpeg_data = cv2.imencode('.jpg', latest_camera_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        jpeg_bytes = jpeg_data.tobytes()
        
        # 연결된 모든 클라이언트에 전송
        for websocket in connected_websockets:
            try:
                await websocket.send_bytes(jpeg_bytes)
            except Exception:
                pass

async def broadcast_ocr_result():
    """모든 연결된 클라이언트에 OCR 결과 전송"""
    if latest_ocr_result and connected_websockets:
        for websocket in connected_websockets:
            try:
                await websocket.send_json(latest_ocr_result)
            except Exception:
                pass

# 웹소켓 엔드포인트
@app.websocket("/ws/camera")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    client_ip = websocket.client.host if hasattr(websocket, 'client') else 'unknown'
    client_port = websocket.client.port if hasattr(websocket, 'client') else 'unknown'
    print(f"[WebSocket] 클라이언트 접속: IP={client_ip}, PORT={client_port}, HEADERS={websocket.headers}")
    logger.info(f"[WebSocket] 클라이언트 접속: IP={client_ip}, PORT={client_port}, HEADERS={websocket.headers}")
    connected_websockets.append(websocket)
    try:
        while True:
            _ = await websocket.receive_text()
            if latest_camera_frame is not None:
                _, jpeg_data = cv2.imencode('.jpg', latest_camera_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                jpeg_bytes = jpeg_data.tobytes()
                await websocket.send_bytes(jpeg_bytes)
    except WebSocketDisconnect:
        print(f"[WebSocket] 클라이언트 연결 해제: IP={client_ip}, PORT={client_port}")
        logger.info(f"[WebSocket] 클라이언트 연결 해제: IP={client_ip}, PORT={client_port}")
        connected_websockets.remove(websocket)

# REST API 엔드포인트
@app.get("/")
async def root():
    """루트 엔드포인트"""
    return {
        "status": "ok",
        "message": "주차 시스템 브릿지 서버가 실행 중입니다.",
        "server_time": datetime.now().isoformat(),
        "uptime": str(datetime.now() - datetime.fromisoformat(server_status["start_time"]))
    }

@app.get("/status")
async def get_status():
    """상태 확인 엔드포인트"""
    try:
        # 서버 상태 업데이트
        server_status["connected_clients"] = len(connected_websockets)
        server_status["last_camera_frame"] = datetime.now().isoformat() if latest_camera_frame is not None else None
        server_status["last_ocr_result"] = latest_ocr_result

        # DB 연결 상태 확인
        parking_stats = db_manager.get_parking_statistics()
        parked_vehicles = []
        parked_df = db_manager.fetch_current_parked_vehicles()
        
        if not parked_df.empty:
            for _, row in parked_df.iterrows():
                vehicle = {
                    "license_plate": row['license_plate'],
                    "car_type": row['car_type'],
                    "location": row['location'],
                    "time": str(row['time'])
                }
                parked_vehicles.append(vehicle)
        
        return {
            "status": "ok",
            "server_status": server_status,
            "timestamp": datetime.now().isoformat(),
            "statistics": parking_stats,
            "parkedVehicles": parked_vehicles,
            "latest_ocr": latest_ocr_result
        }
    except Exception as e:
        logger.error(f"상태 확인 중 오류 발생: {e}")
        raise HTTPException(
            status_code=500,
            detail={
                "status": "error",
                "message": str(e),
                "timestamp": datetime.now().isoformat()
            }
        )

@app.post("/park")
async def park_vehicle(request: ParkingRequest):
    """차량 주차 처리"""
    try:
        # 이미 주차된 차량인지 확인
        vehicle_info = db_manager.get_vehicle_info(request.license_plate)
        if vehicle_info is not None:
            raise HTTPException(status_code=400, detail="이미 주차된 차량입니다.")
        
        # 차량 타입에 맞는 주차 위치 할당
        available_locations = {
            "normal": ["A-1", "A-2"],
            "ev": ["B-1", "B-2"],
            "disabled": ["C-1", "C-2"]
        }
        
        # 주차된 차량 조회
        parked_df = db_manager.fetch_current_parked_vehicles()
        occupied_locations = {}
        
        if not parked_df.empty:
            for _, row in parked_df.iterrows():
                if row['location'] != '-':
                    occupied_locations[row['location']] = row['license_plate']
        
        # 사용 가능한 위치 찾기
        location = "-"
        for loc in available_locations.get(request.car_type, []):
            if loc not in occupied_locations:
                location = loc
                break
        
        if location == "-":
            raise HTTPException(status_code=400, detail="사용 가능한 주차 공간이 없습니다.")
        
        # DB에 주차 기록 추가
        success = db_manager.park_vehicle(request.license_plate, request.car_type, location)
        
        if not success:
            raise HTTPException(status_code=500, detail="주차 기록 추가 실패")
        
        # 위치 정보 발행
        ros_node.publish_location(location)
        
        return {
            "status": "success",
            "license_plate": request.license_plate,
            "car_type": request.car_type,
            "location": location
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"서버 오류: {str(e)}")

@app.post("/exit")
async def exit_vehicle(request: ParkingRequest):
    """차량 출차 처리"""
    try:
        # 주차된 차량인지 확인
        vehicle_info = db_manager.get_vehicle_info(request.license_plate)
        if vehicle_info is None:
            raise HTTPException(status_code=404, detail="주차된 차량을 찾을 수 없습니다.")
        
        car_type = vehicle_info['car_type']
        location = vehicle_info['location']
        
        # 위치 정보 발행
        if location != '-':
            ros_node.publish_location(location)
        
        # DB에 출차 기록 추가
        success = db_manager.exit_vehicle(request.license_plate, car_type, location)
        
        if not success:
            raise HTTPException(status_code=500, detail="출차 기록 추가 실패")
        
        return {
            "status": "success",
            "license_plate": request.license_plate,
            "car_type": car_type,
            "location": location
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"서버 오류: {str(e)}")

@app.post("/publish_location")
async def publish_location(request: LocationRequest):
    """위치 정보 발행"""
    try:
        success = ros_node.publish_location(request.location)
        
        if not success:
            raise HTTPException(status_code=500, detail="위치 정보 발행 실패")
        
        return {
            "status": "success",
            "location": request.location
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"서버 오류: {str(e)}")

@app.get("/latest_ocr")
async def get_latest_ocr():
    """최신 OCR 결과 반환"""
    return latest_ocr_result

@app.get("/camera_frame")
async def get_camera_frame():
    """현재 카메라 프레임을 JPEG로 스트리밍"""
    if latest_camera_frame is None:
        raise HTTPException(status_code=404, detail="카메라 프레임이 없습니다.")
    
    def generate_frames():
        while True:
            if latest_camera_frame is not None:
                # JPEG로 인코딩
                _, jpeg_data = cv2.imencode('.jpg', latest_camera_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg_data.tobytes() + b'\r\n')
            time.sleep(0.033)  # ~30fps
    
    return StreamingResponse(generate_frames(), media_type="multipart/x-mixed-replace; boundary=frame")

# ROS2 스레드 함수
def ros_thread():
    global ros_node
    rclpy.init()
    ros_node = BridgeNode()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # 구글 DNS로 연결 시도(실제 전송 없음)로 로컬 IP 추출
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip

# 메인 함수 수정
if __name__ == "__main__":
    # 메인 이벤트 루프 설정
    main_event_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(main_event_loop)
    
    # ROS2 스레드 시작
    ros_thread = threading.Thread(target=ros_thread, daemon=True)
    ros_thread.start()

    # mDNS/Bonjour(ZeroConf) 서비스 등록
    zeroconf = Zeroconf()
    local_ip = get_local_ip()
    info = ServiceInfo(
        "_parkingapi._tcp.local.",
        "ParkingAPI._parkingapi._tcp.local.",
        addresses=[socket.inet_aton(local_ip)],
        port=8000,
        properties={},
        server="parking.local."
    )
    zeroconf.register_service(info)

    # FastAPI 서버 시작
    uvicorn.run(app, host="0.0.0.0", port=8000) 