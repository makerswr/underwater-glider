from flask import Flask, render_template, Response, send_from_directory, jsonify, request
from flask_socketio import SocketIO
from flask_cors import CORS
import cv2
import time
import threading
import serial
import json
import glob
import os
import subprocess
import re

app = Flask(__name__)
CORS(app)
# 크로스 도메인 허용 명시 (Socket.IO 4 클라이언트와 혼선 방지)
socketio = SocketIO(app, cors_allowed_origins="*")

# --- 영상 스트리밍 성능 최적화(저지연) 공유 상태 ---
cv2.setUseOptimized(True)
latest_jpeg = None
latest_lock = threading.Lock()
camera_thread = None
TARGET_ENCODE_FPS = 20  # 인코딩 목표 FPS
JPEG_QUALITY = int(os.getenv('JPEG_QUALITY', '70'))
ADAPTIVE_JPEG = os.getenv('ADAPTIVE_JPEG', '1').lower() in ('1', 'true', 'yes', 'on')
TARGET_KB_PER_FRAME = float(os.getenv('TARGET_KB_PER_FRAME', '80'))
MIN_JPEG_QUALITY = int(os.getenv('MIN_JPEG_QUALITY', '40'))
MAX_JPEG_QUALITY = int(os.getenv('MAX_JPEG_QUALITY', '85'))
current_jpeg_quality = max(MIN_JPEG_QUALITY, min(MAX_JPEG_QUALITY, JPEG_QUALITY))
last_adapt_time = 0.0
# 스트림 해상도(카메라 해상도와 분리)
STREAM_WIDTH = int(os.getenv('STREAM_WIDTH', '640'))
STREAM_HEIGHT = int(os.getenv('STREAM_HEIGHT', '480'))

# 녹화 품질/코덱 설정
RECORD_PREFERRED_CODEC = (os.getenv('RECORD_CODEC', 'avc1') or 'avc1').upper()
RECORD_FPS = float(os.getenv('RECORD_FPS', '30'))
CAM_WIDTH = int(os.getenv('CAM_WIDTH', '640'))
CAM_HEIGHT = int(os.getenv('CAM_HEIGHT', '480'))

# --- 미션 상태 ---
mission_running = False
mission_config = None
mission_thread = None
current_mission = None
stop_flag = False

# --- 블루투스 SPP 설정 ---
RFCOMM_DEVICE = os.getenv('BT_RFCOMM_DEV', '/dev/rfcomm0')
RFCOMM_BAUD = int(os.getenv('BT_BAUD', '115200'))
bluetooth_thread = None

# --- 아두이노 연결 설정 ---
BAUD_RATE = 115200
ser = None
SERIAL_PORT = None
connection_retry_count = 0
MAX_RETRY_COUNT = 5
RETRY_DELAY = 2  # 초

# --- 비디오 저장 경로 ---
VIDEO_DIR = os.path.join(os.path.dirname(__file__), 'videos')
try:
    os.makedirs(VIDEO_DIR, exist_ok=True)
except Exception:
    pass

def detect_serial_ports():
    ports = []
    common_ports = ['/dev/ttyUSB*','/dev/ttyACM*','/dev/ttyS*','/dev/ttyAMA*','/dev/tty.usb*','/dev/tty.wchusb*']
    for pattern in common_ports:
        ports.extend(glob.glob(pattern))
    valid_ports = []
    for port in ports:
        try:
            if os.path.exists(port) and os.access(port, os.R_OK | os.W_OK):
                valid_ports.append(port)
        except:
            continue
    return sorted(valid_ports)

def test_serial_connection(port):
    try:
        test_ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(2)
        test_ser.write(b'ping\n')
        time.sleep(0.5)
        if test_ser.in_waiting > 0:
            response = test_ser.readline().decode('utf-8', 'ignore').strip()
            test_ser.close()
            return True, response
        else:
            test_ser.close()
            return False, "No response"
    except Exception as e:
        return False, str(e)

def find_arduino_port():
    print("🔍 Searching for Arduino...")
    ports = detect_serial_ports()
    print(f"📋 Found {len(ports)} potential ports: {ports}")
    for port in ports:
        print(f"🔌 Testing port: {port}")
        success, response = test_serial_connection(port)
        if success:
            print(f"✅ Arduino found on {port} - Response: {response}")
            return port
        else:
            print(f"❌ {port}: {response}")
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        if 'Arduino' in result.stdout or 'USB' in result.stdout:
            print("🔍 USB Arduino device detected, but no valid port found")
    except:
        pass
    return None

def connect_to_arduino():
    global ser, SERIAL_PORT, connection_retry_count
    if SERIAL_PORT is None or ser is None or not ser.is_open:
        SERIAL_PORT = find_arduino_port()
    if SERIAL_PORT:
        try:
            if ser and ser.is_open:
                ser.close()
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            connection_retry_count = 0
            print(f"✅ Connected to Arduino on {SERIAL_PORT}")
            return True
        except serial.SerialException as e:
            print(f"❌ Failed to connect to Arduino on {SERIAL_PORT}: {e}")
            SERIAL_PORT = None
            return False
    else:
        print("❌ No Arduino port found")
        return False

def reconnect_arduino():
    global connection_retry_count
    if connection_retry_count < MAX_RETRY_COUNT:
        connection_retry_count += 1
        print(f"🔄 Attempting to reconnect Arduino (attempt {connection_retry_count}/{MAX_RETRY_COUNT})")
        if connect_to_arduino():
            socketio.emit('connection_status', {
                'arduino_connected': True,
                'arduino_port': SERIAL_PORT,
                'message': f'Arduino reconnected on {SERIAL_PORT}'
            })
            return True
        else:
            time.sleep(RETRY_DELAY)
            return False
    else:
        print("❌ Maximum reconnection attempts reached")
        socketio.emit('connection_status', {
            'arduino_connected': False,
            'arduino_port': 'Not found',
            'message': 'Arduino connection failed after multiple attempts'
        })
        return False

# 초기 연결 시도
connect_to_arduino()

# --- 직렬 송신 헬퍼 ---
def send_serial_command(command_line: str) -> bool:
    try:
        if not command_line.endswith('\n'):
            command_line = f"{command_line}\n"
        if not ser or not ser.is_open:
            print("⚠️ Arduino not connected - attempting reconnection before sending command")
            if not reconnect_arduino():
                socketio.emit('serial_log', {'data': '[ERROR] Cannot send command - Arduino not connected'})
                return False
        ser.write(command_line.encode('utf-8'))
        log_message = f"[SENT] {command_line.strip()}"
        print(log_message)
        socketio.emit('serial_log', {'data': log_message})
        return True
    except Exception as e:
        error_msg = f"[ERROR] Failed to send command: {e}"
        print(error_msg)
        socketio.emit('serial_log', {'data': error_msg})
        if "device not found" in str(e).lower() or "permission denied" in str(e).lower():
            print("🔄 Connection error during command send - attempting reconnection")
            reconnect_arduino()
        return False

# 미션용 간단 헬퍼: m1/m2 정수 스텝 전송(+미션 로그)
def send_cmd(m1: int, m2: int) -> bool:
    global ser
    try:
        serial_line = f"{int(m1)},{int(m2)}\n"
        if not ser or not ser.is_open:
            print("⚠️ Arduino not connected - attempting reconnection before sending command")
            if not reconnect_arduino():
                socketio.emit('serial_log', {'data': '[MISSION] ERROR: Cannot send command - Arduino not connected'})
                return False
        ser.write(serial_line.encode('utf-8'))
        socketio.emit('serial_log', {'data': f"[MISSION] Sent: {serial_line.strip()}"})
        return True
    except Exception as e:
        socketio.emit('serial_log', {'data': f"[MISSION] ERROR sending command: {e}"})
        return False

# --- 카메라 초기화 ---
camera = None
camera_status = False
camera_device = None
latest_frame = None
# 녹화 상태
record_lock = threading.Lock()
recording = False
record_writer = None
record_filename = None
record_frames = 0
record_start_time = 0.0

def find_working_camera():
    """사용 가능한 카메라 장치를 찾습니다."""
    import os
    
    # 가능한 카메라 장치들 (우선순위 순)
    camera_candidates = [
        # 일반적인 카메라 인덱스
        0, 1, 2, 3, 4,
        # Raspberry Pi 카메라 모듈
        '/dev/video0', '/dev/video1', '/dev/video2', '/dev/video3',
        # USB 카메라 (일반적으로 높은 번호)
        '/dev/video10', '/dev/video11', '/dev/video12', '/dev/video13',
        '/dev/video14', '/dev/video15', '/dev/video16', '/dev/video17',
        '/dev/video18', '/dev/video19', '/dev/video20', '/dev/video21',
        '/dev/video22', '/dev/video23', '/dev/video24', '/dev/video25',
        '/dev/video26', '/dev/video27', '/dev/video28', '/dev/video29',
        '/dev/video30', '/dev/video31'
    ]
    
    print("🔍 Searching for working camera...")
    
    for device in camera_candidates:
        try:
            print(f"  Testing camera: {device}")
            
            # V4L2 백엔드로 시도
            if isinstance(device, str) and device.startswith('/dev/video'):
                test_camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
            else:
                test_camera = cv2.VideoCapture(device)
            
            if test_camera.isOpened():
                # 실제로 프레임을 읽을 수 있는지 테스트
                ret, frame = test_camera.read()
                if ret and frame is not None and frame.size > 0:
                    print(f"✅ Working camera found: {device}")
                    test_camera.release()
                    return device
                else:
                    print(f"  Camera {device}: Opened but no valid frame")
            else:
                print(f"  Camera {device}: Cannot open")
            
            test_camera.release()
            
        except Exception as e:
            print(f"  Camera {device}: Error - {e}")
            continue
    
    print("❌ No working camera found")
    return None

def initialize_camera():
    """카메라를 초기화합니다."""
    global camera, camera_status, camera_device
    
    camera_device = find_working_camera()
    
    if camera_device is None:
        print("⚠️ No camera available - continuing without camera")
        camera_status = False
        return False
    
    try:
        # V4L2 백엔드 사용
        if isinstance(camera_device, str) and camera_device.startswith('/dev/video'):
            camera = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
        else:
            camera = cv2.VideoCapture(camera_device)
            
        if camera.isOpened():
            # 카메라로부터 MJPG 포맷을 요청(지원 시 CPU 부하 감소)
            try:
                camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            except Exception:
                pass
            # 카메라 설정
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
            camera.set(cv2.CAP_PROP_FPS, 30)
            camera.set(cv2.CAP_PROP_BUFFERSIZE, 3)  # 너무 작은 버퍼로 인한 교착 방지
            
            # 실제 프레임 읽기 테스트
            ret, frame = camera.read()
            if ret and frame is not None and frame.size > 0:
                camera_status = True
                print(f"✅ Camera initialized successfully: {camera_device}")
                print(f"   Frame size: {frame.shape}")
                # 백그라운드 캡처 스레드 시작
                try:
                    start_camera_capture_thread()
                except NameError:
                    # 함수 정의 순서로 인한 임시 예외 방지
                    pass
                return True
            else:
                print(f"⚠️ Camera opened but cannot read frames: {camera_device}")
                camera.release()
                camera = None
                camera_status = False
                return False
        else:
            print(f"⚠️ Failed to open camera: {camera_device}")
            camera_status = False
            return False
            
    except Exception as e:
        print(f"⚠️ Camera initialization failed: {e}")
        camera_status = False
        return False

# --- 저지연 캡처/프리인코딩 스레드 ---
def camera_capture_worker():
    global latest_jpeg, latest_frame, recording, record_writer, record_frames, current_jpeg_quality, last_adapt_time
    last_encode = 0.0
    encode_interval = 1.0 / max(1, TARGET_ENCODE_FPS)
    while True:
        try:
            if camera and camera_status and camera.isOpened():
                success, frame = camera.read()
                if not success or frame is None:
                    time.sleep(0.005)
                    continue
                # 최신 프레임 보관 (녹화/기타 용도)
                with latest_lock:
                    latest_frame = frame
                now = time.time()
                if now - last_encode < encode_interval:
                    # 인코딩 타이밍이 아니어도 녹화는 수행
                    with record_lock:
                        if recording and record_writer is not None:
                            try:
                                record_writer.write(frame)
                                record_frames += 1
                            except Exception:
                                pass
                    # 바쁜 루프 방지
                    time.sleep(0.001)
                    continue
                # 스트림 해상도로 다운스케일 후 전송(카메라 해상도와 분리)
                stream_frame = frame
                try:
                    if STREAM_WIDTH > 0 and STREAM_HEIGHT > 0:
                        h, w = frame.shape[:2]
                        if w != STREAM_WIDTH or h != STREAM_HEIGHT:
                            stream_frame = cv2.resize(frame, (STREAM_WIDTH, STREAM_HEIGHT), interpolation=cv2.INTER_AREA)
                except Exception:
                    stream_frame = frame
                q = int(current_jpeg_quality)
                ok, buf = cv2.imencode('.jpg', stream_frame, [int(cv2.IMWRITE_JPEG_QUALITY), q])
                if ok:
                    data = buf.tobytes()
                    with latest_lock:
                        latest_jpeg = data
                    last_encode = now
                    if ADAPTIVE_JPEG:
                        try:
                            size_kb = len(data) / 1024.0
                            if (now - float(last_adapt_time or 0.0)) > 0.3:
                                new_q = q
                                if size_kb > TARGET_KB_PER_FRAME * 1.2:
                                    new_q = max(MIN_JPEG_QUALITY, q - 5)
                                elif size_kb < TARGET_KB_PER_FRAME * 0.6:
                                    new_q = min(MAX_JPEG_QUALITY, q + 2)
                                if new_q != q:
                                    current_jpeg_quality = new_q
                                last_adapt_time = now
                        except Exception:
                            pass
                # 녹화 중이면 프레임을 파일에 기록
                with record_lock:
                    if recording and record_writer is not None:
                        try:
                            record_writer.write(frame)
                            record_frames += 1
                        except Exception:
                            pass
            else:
                time.sleep(0.1)
        except Exception as e:
            try:
                socketio.emit('serial_log', {'data': f"[VIDEO] capture error: {e}"})
            except Exception:
                pass
            time.sleep(0.02)

def start_camera_capture_thread():
    global camera_thread
    if camera_thread and camera_thread.is_alive():
        return
    camera_thread = threading.Thread(target=camera_capture_worker, daemon=True)
    camera_thread.start()

# --- 블루투스 SPP 브리지 ---
def bluetooth_bridge_worker():
    """
    /dev/rfcomm0에 연결된 안드로이드 SPP로부터 "M1,M2" 텍스트 라인을 수신하여
    기존 직렬 송신 헬퍼를 통해 아두이노로 중계한다.
    """
    pattern = re.compile(r'^\s*[+-]?\d+\s*,\s*[+-]?\d+\s*$')
    bt_ser = None
    last_log_time = 0.0
    while True:
        try:
            if bt_ser is None:
                if os.path.exists(RFCOMM_DEVICE):
                    try:
                        bt_ser = serial.Serial(RFCOMM_DEVICE, RFCOMM_BAUD, timeout=0.2)
                        socketio.emit('serial_log', {'data': f'[BT] Connected to {RFCOMM_DEVICE} ({RFCOMM_BAUD}bps)'})
                    except Exception as e:
                        bt_ser = None
                        if time.time() - last_log_time > 5:
                            socketio.emit('serial_log', {'data': f'[BT] Open error on {RFCOMM_DEVICE}: {e}'})
                            last_log_time = time.time()
                        time.sleep(1.0)
                        continue
                else:
                    # rfcomm 장치가 없으면 주기적으로 안내
                    if time.time() - last_log_time > 10:
                        socketio.emit('serial_log', {'data': f'[BT] Waiting for {RFCOMM_DEVICE}. Pair phone and run SPP (sdptool add SP; rfcomm listen/watch).'})
                        last_log_time = time.time()
                    time.sleep(1.0)
                    continue
            # 데이터 수신
            try:
                line = bt_ser.readline().decode('utf-8', 'ignore').strip()
            except Exception as e:
                socketio.emit('serial_log', {'data': f'[BT] Read error: {e}'})
                try:
                    bt_ser.close()
                except Exception:
                    pass
                bt_ser = None
                time.sleep(0.5)
                continue
            if not line:
                continue
            # 유효성 검사
            if pattern.match(line):
                # 그대로 아두이노로 중계
                send_serial_command(line)
            else:
                socketio.emit('serial_log', {'data': f'[BT] Invalid format: "{line}" (expected "M1,M2")'})
        except Exception as e:
            # 예외는 로깅 후 재시도
            try:
                socketio.emit('serial_log', {'data': f'[BT] Bridge error: {e}'})
            except Exception:
                pass
            time.sleep(0.5)

def start_bluetooth_bridge_thread():
    global bluetooth_thread
    if bluetooth_thread and bluetooth_thread.is_alive():
        return
    bluetooth_thread = threading.Thread(target=bluetooth_bridge_worker, daemon=True)
    bluetooth_thread.start()

# 카메라 초기화 실행
initialize_camera()

# --- 백그라운드 스레드 (아두이노 데이터 수신) ---
def arduino_reader_thread():
    last_heartbeat = time.time()
    heartbeat_timeout = 10
    # sensor_update 송신 스로틀(실시간, 60Hz)
    last_emit = 0.0
    emit_interval = 1.0 / 60.0

    while True:
        try:
            if ser and ser.is_open:
                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode('utf-8', 'ignore').strip()
                        if line:
                            last_heartbeat = time.time()
                            # 센서 데이터는 실시간으로 전송, 로그만 5초마다
                            if line.startswith('{') and line.endswith('}'):
                                now = time.time()
                                if now - last_emit >= emit_interval:
                                    sensor_data = json.loads(line)
                                    socketio.emit('sensor_update', sensor_data)
                                    last_emit = now
                            else:
                                # JSON이 아닌 메시지만 로그 출력
                                socketio.emit('serial_log', {'data': f'[RECV] {line}'})
                    except Exception as e:
                        socketio.emit('serial_log', {'data': f"[ERROR] {e}"})
                        if "device not found" in str(e).lower() or "permission denied" in str(e).lower():
                            print(f"⚠️ Connection error detected: {e}")
                            if reconnect_arduino():
                                last_heartbeat = time.time()
                            else:
                                time.sleep(5)
                else:
                    if time.time() - last_heartbeat > heartbeat_timeout:
                        print("⚠️ Arduino heartbeat timeout - attempting reconnection")
                        if reconnect_arduino():
                            last_heartbeat = time.time()
                        else:
                            time.sleep(5)
            else:
                print("⚠️ Arduino not connected - attempting reconnection")
                if reconnect_arduino():
                    last_heartbeat = time.time()
                else:
                    time.sleep(5)
        except Exception as e:
            print(f"❌ Arduino reader thread error: {e}")
            socketio.emit('serial_log', {'data': f"[THREAD ERROR] {e}"})
            time.sleep(1)
        time.sleep(0.001)

def generate_frames():
    import numpy as np
    prev_payload = None
    while True:
        with latest_lock:
            payload = latest_jpeg
        if payload:
            # 새 프레임이 올 때만 전송(중복 전송 방지)
            if payload is prev_payload:
                time.sleep(0.002)
                continue
            header = (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n'
                b'Content-Length: ' + str(len(payload)).encode('ascii') + b'\r\n\r\n'
            )
            yield header + payload + b'\r\n'
            prev_payload = payload
            continue
        # 초기 대기 또는 카메라 미가용 시 플레이스홀더 출력
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(frame, "Camera Not Available", (200, 240),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame, "Glider Control System Active", (150, 280),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
        ret, buffer = cv2.imencode('.jpg', frame)
        if ret:
            frame_bytes = buffer.tobytes()
            header = (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n'
                b'Content-Length: ' + str(len(frame_bytes)).encode('ascii') + b'\r\n\r\n'
            )
            yield header + frame_bytes + b'\r\n'
        time.sleep(0.1)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    headers = {
        'Cache-Control': 'no-cache, no-store, must-revalidate, no-transform',
        'Pragma': 'no-cache',
        'Expires': '0',
        'X-Accel-Buffering': 'no'
    }
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame', headers=headers)

@app.get('/videos')
def list_videos():
    try:
        files = []
        for name in os.listdir(VIDEO_DIR):
            if name.lower().endswith(('.mp4', '.avi', '.mov', '.mkv')):
                path = os.path.join(VIDEO_DIR, name)
                try:
                    stat = os.stat(path)
                    files.append({
                        'name': name,
                        'size_bytes': stat.st_size,
                        'mtime': stat.st_mtime
                    })
                except Exception:
                    continue
        # 최신순 정렬
        files.sort(key=lambda x: x['mtime'], reverse=True)
        return jsonify({'videos': files})
    except Exception as e:
        return jsonify({'error': str(e), 'videos': []}), 500

@app.get('/videos/<path:filename>')
def serve_video(filename):
    # 단순한 파일 서빙 (보안: 상위 경로 이동 방지)
    return send_from_directory(VIDEO_DIR, filename, as_attachment=False)

@socketio.on('connect')
def handle_connect():
    print('Client connected!')
    connection_status = {
        'arduino_connected': ser is not None and ser.is_open,
        'camera_connected': camera_status,
        'arduino_port': SERIAL_PORT if ser and ser.is_open else 'Not connected',
        'camera_device': camera_device if camera_status else 'Not available',
        'camera_status': 'Connected' if camera_status else 'Not available',
        'retry_count': connection_retry_count,
        'max_retry_count': MAX_RETRY_COUNT
    }
    socketio.emit('connection_status', connection_status)
    global thread
    if 'thread' not in globals() or not thread.is_alive():
        thread = threading.Thread(target=arduino_reader_thread, daemon=True)
        thread.start()
    # 블루투스 브리지도 필요 시 시작
    if 'bluetooth_thread' in globals():
        start_bluetooth_bridge_thread()

@socketio.on('request_reconnect')
def handle_reconnect_request():
    print('🔄 Manual reconnection requested by client')
    global connection_retry_count
    connection_retry_count = 0
    
    # 아두이노 재연결
    arduino_success = connect_to_arduino()
    
    # 카메라 재연결
    camera_success = initialize_camera()
    
    if arduino_success:
        socketio.emit('connection_status', {
            'arduino_connected': True,
            'arduino_port': SERIAL_PORT,
            'camera_connected': camera_status,
            'camera_device': camera_device if camera_status else 'Not available',
            'message': f'Arduino manually reconnected on {SERIAL_PORT}',
            'retry_count': connection_retry_count,
            'max_retry_count': MAX_RETRY_COUNT
        })
        socketio.emit('serial_log', {'data': f'[SYSTEM] Manual reconnection successful on {SERIAL_PORT}'})
    else:
        socketio.emit('connection_status', {
            'arduino_connected': False,
            'arduino_port': 'Not found',
            'camera_connected': camera_status,
            'camera_device': camera_device if camera_status else 'Not available',
            'message': 'Manual reconnection failed - Arduino not found',
            'retry_count': connection_retry_count,
            'max_retry_count': MAX_RETRY_COUNT
        })
        socketio.emit('serial_log', {'data': '[SYSTEM] Manual reconnection failed - Arduino not found'})
    
    if camera_success:
        socketio.emit('serial_log', {'data': f'[SYSTEM] Camera reconnected: {camera_device}'})
    else:
        socketio.emit('serial_log', {'data': '[SYSTEM] Camera reconnection failed'})

@socketio.on('request_port_scan')
def handle_port_scan_request():
    print('🔍 Port scan requested by client')
    ports = detect_serial_ports()
    port_info = []
    for port in ports:
        success, response = test_serial_connection(port)
        port_info.append({'port': port, 'available': success, 'response': response})
    socketio.emit('port_scan_results', {'ports': port_info, 'message': f'Found {len(ports)} potential ports'})
    socketio.emit('serial_log', {'data': f'[SYSTEM] Port scan completed - {len(ports)} ports found'})

@socketio.on('control_event')
def handle_control_event(json_data):
    command_to_send = json_data.get('command')
    if command_to_send:
        send_serial_command(command_to_send)

@socketio.on('request_simulation')
def handle_simulation_request():
    import random
    mock_data = {
        'accX': round(random.uniform(-2, 2), 2),
        'accY': round(random.uniform(-2, 2), 2),
        'accZ': round(random.uniform(8, 12), 2),
        'gyroX': round(random.uniform(-100, 100), 2),
        'gyroY': round(random.uniform(-100, 100), 2),
        'gyroZ': round(random.uniform(-100, 100), 2),
        'temperature': round(random.uniform(20, 30), 1),
        'pitch': round(random.uniform(-20, 20), 1),
        'source': 'SIM'
    }
    socketio.emit('sensor_update', mock_data)
    print(f"[SIMULATION] Mock sensor data sent: {mock_data}")

def _coerce_int(v, default=0):
    try:
        return int(float(v))
    except Exception:
        return int(default)

def _coerce_float(v, default=0.0):
    try:
        return float(v)
    except Exception:
        return float(default)

def build_segments_from_pattern(pattern: str, params: dict):
    """
    패턴과 파라미터를 입력으로 받아 실행 가능한 세그먼트 목록과 start_delay를 생성합니다.
    각 세그먼트는 {'m1': int, 'm2': int, 'wait': float, 'label': str} 형태입니다.
    """
    pattern = (pattern or '').strip()
    p = params or {}
    segments = []
    start_delay = _coerce_float(p.get('start_delay', p.get('delay', 0.0)), 0.0)

    if pattern == 'sawtooth_pitch':
        pitch_step = _coerce_int(p.get('pitch_step', 200), 200)
        segment_time = max(0.0, _coerce_float(p.get('segment_time', 2.0), 2.0))
        repeat = max(0, _coerce_int(p.get('repeat', 5), 5))
        for i in range(1, repeat + 1):
            segments.append({'m1': +pitch_step, 'm2': 0, 'wait': segment_time, 'label': f"Cycle {i}/{repeat}: command {+pitch_step},0"})
            segments.append({'m1': -pitch_step, 'm2': 0, 'wait': segment_time, 'label': f"Cycle {i}/{repeat}: command {-pitch_step},0"})
        return segments, start_delay

    if pattern == 'pitch_only_test':
        pitch_step = _coerce_int(p.get('pitch_step', 200), 200)
        segment_time = max(0.0, _coerce_float(p.get('segment_time', 2.0), 2.0))
        segments.append({'m1': +pitch_step, 'm2': 0, 'wait': segment_time, 'label': f"Pitch test: command {+pitch_step},0"})
        segments.append({'m1': -pitch_step, 'm2': 0, 'wait': segment_time, 'label': f"Pitch test: command {-pitch_step},0"})
        return segments, start_delay

    if pattern == 'small_box_turn':
        pitch_step = _coerce_int(p.get('pitch_step', 200), 200)
        turn_step = _coerce_int(p.get('turn_step', 200), 200)
        seg_fwd = max(0.0, _coerce_float(p.get('segment_time_forward', 2.0), 2.0))
        seg_turn = max(0.0, _coerce_float(p.get('segment_time_turn', 1.0), 1.0))
        repeat = max(0, _coerce_int(p.get('repeat', 4), 4))
        for i in range(1, repeat + 1):
            segments.append({'m1': +pitch_step, 'm2': 0, 'wait': seg_fwd,  'label': f"Cycle {i}/{repeat}: forward +{pitch_step},0"})
            segments.append({'m1': 0,           'm2': +turn_step, 'wait': seg_turn, 'label': f"Cycle {i}/{repeat}: turn 0,+{turn_step}"})
            segments.append({'m1': -pitch_step, 'm2': 0, 'wait': seg_fwd,  'label': f"Cycle {i}/{repeat}: forward -{pitch_step},0"})
            segments.append({'m1': 0,           'm2': +turn_step, 'wait': seg_turn, 'label': f"Cycle {i}/{repeat}: turn 0,+{turn_step}"})
        return segments, start_delay

    # 미지원 패턴
    return None, start_delay

@socketio.on('start_mission')
def handle_start_mission(mission_json):
    global mission_running, mission_thread, current_mission, stop_flag
    try:
        if not isinstance(mission_json, dict):
            socketio.emit('serial_log', {'data': '[MISSION] Invalid mission JSON'})
            return
        pattern = (mission_json.get('pattern') or '').strip()
        socketio.emit('serial_log', {'data': f'[MISSION] Start requested: pattern={pattern}'})
        if mission_running:
            socketio.emit('serial_log', {'data': '[MISSION] Already running - rejecting new start'})
            return

        # custom_segments 모드
        if pattern == 'custom_segments':
            segments_list = mission_json.get('segments') or []
            if not isinstance(segments_list, list) or len(segments_list) == 0:
                socketio.emit('serial_log', {'data': '[MISSION] custom_segments requires non-empty "segments" array'})
                return
            # 세그먼트 정규화
            norm_segments = []
            for idx, seg in enumerate(segments_list, start=1):
                try:
                    m1 = _coerce_int(seg.get('m1', 0), 0)
                    m2 = _coerce_int(seg.get('m2', 0), 0)
                    w = max(0.0, _coerce_float(seg.get('wait', 0.0), 0.0))
                    norm_segments.append({'m1': m1, 'm2': m2, 'wait': w, 'label': f"Segment {idx}: command {m1},{m2}"})
                except Exception:
                    socketio.emit('serial_log', {'data': f'[MISSION] Invalid segment at index {idx-1}'})
                    return
            start_delay = max(0.0, _coerce_float(mission_json.get('start_delay', 0.0), 0.0))
            current_mission = {'pattern': 'custom_segments', 'segments': norm_segments, 'start_delay': start_delay}
        else:
            # 패턴 기반: params 또는 구 포맷 호환
            params = mission_json.get('params')
            if not isinstance(params, dict):
                # 구(이전) 포맷 호환: 상위 키에서 파라미터 읽기
                params = {k: mission_json.get(k) for k in ['pitch_step','segment_time','start_delay','repeat','turn_step','segment_time_forward','segment_time_turn'] if k in mission_json}
            segments, start_delay = build_segments_from_pattern(pattern, params)
            if not segments:
                socketio.emit('serial_log', {'data': f'[MISSION] Unsupported or invalid pattern: {pattern}'})
                return
            current_mission = {'pattern': pattern, 'segments': segments, 'start_delay': max(0.0, _coerce_float(start_delay, 0.0))}

        # 워커 스레드 시작
        stop_flag = False
        mission_thread = threading.Thread(target=mission_worker, args=(current_mission,), daemon=True)
        mission_thread.start()
    except Exception as e:
        socketio.emit('serial_log', {'data': f'[MISSION] Error: {e}'})

def mission_worker(mission_cfg: dict):
    global mission_running, stop_flag
    mission_running = True
    try:
        pattern = mission_cfg.get('pattern')
        segments = mission_cfg.get('segments') or []
        start_delay = float(mission_cfg.get('start_delay') or 0.0)
        socketio.emit('serial_log', {'data': f"[MISSION] Starting {pattern}: segments={len(segments)}, start_delay={start_delay}"})
        if start_delay > 0:
            socketio.emit('serial_log', {'data': f"[MISSION] Waiting {start_delay:.1f}s before first command"})
            time.sleep(start_delay)
        for seg in segments:
            if stop_flag:
                socketio.emit('serial_log', {'data': "[MISSION] Stopped"})
                break
            label = seg.get('label') or f"Command {seg.get('m1',0)},{seg.get('m2',0)}"
            socketio.emit('serial_log', {'data': f"[MISSION] {label}"})
            send_cmd(seg.get('m1', 0), seg.get('m2', 0))
            wait_s = float(seg.get('wait', 0.0))
            if wait_s > 0:
                time.sleep(wait_s)
                socketio.emit('serial_log', {'data': f"[MISSION] Waited {wait_s:.1f}s"})
        else:
            socketio.emit('serial_log', {'data': f"[MISSION] Completed {pattern}"})
    except Exception as e:
        socketio.emit('serial_log', {'data': f"[MISSION] Runtime error: {e}"})
    finally:
        mission_running = False

@socketio.on('start_recording')
def handle_start_recording():
    global recording, record_writer, record_filename, record_frames, record_start_time
    if not camera_status or camera is None or not camera.isOpened():
        socketio.emit('serial_log', {'data': '[RECORD] ERROR: Camera not available'})
        socketio.emit('recording_status', {'recording': False, 'message': 'Camera not available'})
        return
    def _try_open_writer(base_path: str, width: int, height: int, fps: float):
        """
        선호 코덱(RECORD_CODEC) 우선, 실패 시 폴백 순서대로 시도.
        반환: (writer, filename) 혹은 (None, None)
        """
        prefer = RECORD_PREFERRED_CODEC.upper() if isinstance(RECORD_PREFERRED_CODEC, str) else 'AVC1'
        # 코덱 후보 테이블: (fourcc, ext)
        codec_map = {
            'AVC1': ('avc1', '.mp4'),
            'H264': ('H264', '.mp4'),
            'X264': ('X264', '.mp4'),
            'MP4V': ('mp4v', '.mp4'),
            'MJPG': ('MJPG', '.avi'),
            'XVID': ('XVID', '.avi'),
        }
        order = [prefer, 'H264', 'X264', 'MP4V', 'MJPG', 'XVID']
        for key in order:
            key_up = key.upper()
            if key_up not in codec_map:
                continue
            fourcc_key, ext = codec_map[key_up]
            fourcc = cv2.VideoWriter_fourcc(*fourcc_key)
            filename = base_path + ext
            try:
                w = cv2.VideoWriter(filename, fourcc, float(fps), (int(width), int(height)))
                if w is not None and w.isOpened():
                    socketio.emit('serial_log', {'data': f'[RECORD] Using codec {key_up} -> {os.path.basename(filename)}'})
                    return w, filename
            except Exception as e:
                socketio.emit('serial_log', {'data': f'[RECORD] Codec {key_up} failed: {e}'})
        return None, None
    with record_lock:
        if recording:
            socketio.emit('serial_log', {'data': f'[RECORD] Already recording: {record_filename}'})
            socketio.emit('recording_status', {'recording': True, 'filename': os.path.basename(record_filename)})
            return
        # 프레임 크기/초당프레임 설정
        width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH) or CAM_WIDTH or 640)
        height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT) or CAM_HEIGHT or 480)
        fps_cap = float(camera.get(cv2.CAP_PROP_FPS) or 0.0)
        fps = float(RECORD_FPS if RECORD_FPS > 0 else (fps_cap if fps_cap > 0 else 30.0))
        # 파일명
        ts = time.strftime('%Y%m%d_%H%M%S')
        base = os.path.join(VIDEO_DIR, f'glider_{ts}')
        record_writer, record_filename = _try_open_writer(base, width, height, fps)
        if record_writer is None or not record_writer.isOpened():
            socketio.emit('serial_log', {'data': '[RECORD] ERROR: Cannot open any VideoWriter (tried avc1/h264/x264/mp4v/mjpg/xvid)'})
            socketio.emit('recording_status', {'recording': False, 'message': 'No suitable codec'})
            return
        recording = True
        record_frames = 0
        record_start_time = time.time()
        socketio.emit('serial_log', {'data': f'[RECORD] Started: {os.path.basename(record_filename)} ({width}x{height}@{fps:.1f}fps)'})
        socketio.emit('recording_status', {
            'recording': True,
            'filename': os.path.basename(record_filename),
            'frames': record_frames
        })

@socketio.on('stop_recording')
def handle_stop_recording():
    global recording, record_writer, record_filename, record_frames, record_start_time
    with record_lock:
        if not recording:
            socketio.emit('serial_log', {'data': '[RECORD] Not recording'})
            socketio.emit('recording_status', {'recording': False})
            return
        recording = False
        try:
            if record_writer is not None:
                record_writer.release()
        except Exception:
            pass
        duration = max(0.0, time.time() - float(record_start_time or 0.0))
        fname = os.path.basename(record_filename) if record_filename else '(unknown)'
        # 프레임 없으면 파일 삭제
        if record_frames <= 0 and record_filename and os.path.exists(record_filename):
            try:
                os.remove(record_filename)
                socketio.emit('serial_log', {'data': f'[RECORD] No frames captured. File removed: {fname}'})
            except Exception:
                pass
        else:
            socketio.emit('serial_log', {'data': f'[RECORD] Stopped: {fname} ({record_frames} frames, {duration:.1f}s)'})
        record_writer = None
        record_filename = None
        record_frames = 0
        record_start_time = 0.0
        socketio.emit('recording_status', {'recording': False})

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=1234, allow_unsafe_werkzeug=True)
