from flask import Flask, render_template, Response
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

# --- 미션 상태 ---
mission_running = False
mission_config = None
mission_thread = None

# --- 아두이노 연결 설정 ---
BAUD_RATE = 115200
ser = None
SERIAL_PORT = None
connection_retry_count = 0
MAX_RETRY_COUNT = 5
RETRY_DELAY = 2  # 초

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

# --- 카메라 초기화 ---
camera = None
camera_status = False
camera_device = None

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
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            camera.set(cv2.CAP_PROP_FPS, 30)
            camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 버퍼 크기 최소화
            
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
    global latest_jpeg
    last_encode = 0.0
    encode_interval = 1.0 / max(1, TARGET_ENCODE_FPS)
    while True:
        try:
            if camera and camera_status and camera.isOpened():
                success, frame = camera.read()
                if not success or frame is None:
                    time.sleep(0.005)
                    continue
                now = time.time()
                if now - last_encode < encode_interval:
                    continue
                ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(JPEG_QUALITY)])
                if ok:
                    data = buf.tobytes()
                    with latest_lock:
                        latest_jpeg = data
                    last_encode = now
            else:
                time.sleep(0.1)
        except Exception:
            time.sleep(0.02)

def start_camera_capture_thread():
    global camera_thread
    if camera_thread and camera_thread.is_alive():
        return
    camera_thread = threading.Thread(target=camera_capture_worker, daemon=True)
    camera_thread.start()

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
    while True:
        with latest_lock:
            payload = latest_jpeg
        if payload:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + payload + b'\r\n')
            time.sleep(max(0.0, 1.0 / max(1, TARGET_ENCODE_FPS)))
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
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.1)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    headers = {
        'Cache-Control': 'no-cache, no-store, must-revalidate',
        'Pragma': 'no-cache',
        'Expires': '0',
        'X-Accel-Buffering': 'no'
    }
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame', headers=headers)

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

@socketio.on('start_mission')
def handle_start_mission(cfg):
    global mission_running, mission_config, mission_thread
    try:
        socketio.emit('serial_log', {'data': '[MISSION] Start requested'})
        if mission_running:
            socketio.emit('serial_log', {'data': '[MISSION] Already running - rejecting new start'}); 
            return
        if not isinstance(cfg, dict):
            socketio.emit('serial_log', {'data': '[MISSION] Invalid config: not a dict'})
            return
        pattern = (cfg.get('pattern') or '').strip()
        if pattern != 'sawtooth_pitch':
            socketio.emit('serial_log', {'data': f"[MISSION] Unsupported pattern: {pattern}"})
            return
        # 파라미터 파싱/검증
        try:
            pitch_step = int(cfg.get('pitch_step', 200))
            segment_time = float(cfg.get('segment_time', 2.0))
            start_delay = float(cfg.get('start_delay', 20.0))
            repeat = int(cfg.get('repeat', 5))
        except Exception:
            socketio.emit('serial_log', {'data': '[MISSION] Invalid parameter types'})
            return
        mission_config = {
            'pattern': 'sawtooth_pitch',
            'pitch_step': pitch_step,
            'segment_time': max(0.0, segment_time),
            'start_delay': max(0.0, start_delay),
            'repeat': max(0, repeat),
        }
        # 워커 스레드 시작
        mission_thread = threading.Thread(target=mission_worker, args=(mission_config,), daemon=True)
        mission_thread.start()
    except Exception as e:
        socketio.emit('serial_log', {'data': f'[MISSION] Error: {e}'})

def mission_worker(cfg):
    global mission_running
    mission_running = True
    try:
        step = cfg['pitch_step']; seg = cfg['segment_time']; delay = cfg['start_delay']; rep = cfg['repeat']
        socketio.emit('serial_log', {'data': f"[MISSION] Starting sawtooth_pitch: step={step}, segment_time={seg}, repeat={rep}, start_delay={delay}"})
        if delay > 0:
            socketio.emit('serial_log', {'data': f"[MISSION] Waiting {delay:.1f}s before dive"})
            time.sleep(delay)
        for i in range(1, rep + 1):
            cmd1 = f"{+step},0"
            socketio.emit('serial_log', {'data': f"[MISSION] Cycle {i}/{rep}: command {cmd1}"})
            send_serial_command(cmd1)
            time.sleep(seg)
            cmd2 = f"{-step},0"
            socketio.emit('serial_log', {'data': f"[MISSION] Cycle {i}/{rep}: command {cmd2}"})
            send_serial_command(cmd2)
            time.sleep(seg)
        socketio.emit('serial_log', {'data': "[MISSION] Completed sawtooth_pitch mission"})
    except Exception as e:
        socketio.emit('serial_log', {'data': f"[MISSION] Runtime error: {e}"})
    finally:
        mission_running = False

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=1234, allow_unsafe_werkzeug=True)
