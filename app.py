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

# --- 카메라 초기화 ---
camera = None
camera_status = False
try:
    camera = cv2.VideoCapture(0)
    if camera.isOpened():
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        camera_status = True
        print("✅ Camera initialized.")
    else:
        print("⚠️ Camera not available - continuing without camera")
        camera_status = False
except Exception as e:
    print(f"⚠️ Camera initialization failed: {e} - continuing without camera")
    camera_status = False

# --- 백그라운드 스레드 (아두이노 데이터 수신) ---
def arduino_reader_thread():
    last_heartbeat = time.time()
    heartbeat_timeout = 10
    # sensor_update 송신 스로틀(최대 60Hz)
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
                            socketio.emit('serial_log', {'data': f'[RECV] {line}'})
                            if line.startswith('{') and line.endswith('}'):
                                now = time.time()
                                if now - last_emit >= emit_interval:
                                    sensor_data = json.loads(line)
                                    socketio.emit('sensor_update', sensor_data)
                                    last_emit = now
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
        if camera and camera_status and camera.isOpened():
            success, frame = camera.read()
            if not success:
                time.sleep(0.1); continue
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        else:
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
            time.sleep(1)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('connect')
def handle_connect():
    print('Client connected!')
    connection_status = {
        'arduino_connected': ser is not None and ser.is_open,
        'camera_connected': camera_status,
        'arduino_port': SERIAL_PORT if ser and ser.is_open else 'Not connected',
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
    if connect_to_arduino():
        socketio.emit('connection_status', {
            'arduino_connected': True,
            'arduino_port': SERIAL_PORT,
            'message': f'Arduino manually reconnected on {SERIAL_PORT}',
            'retry_count': connection_retry_count,
            'max_retry_count': MAX_RETRY_COUNT
        })
        socketio.emit('serial_log', {'data': f'[SYSTEM] Manual reconnection successful on {SERIAL_PORT}'})
    else:
        socketio.emit('connection_status', {
            'arduino_connected': False,
            'arduino_port': 'Not found',
            'message': 'Manual reconnection failed - Arduino not found',
            'retry_count': connection_retry_count,
            'max_retry_count': MAX_RETRY_COUNT
        })
        socketio.emit('serial_log', {'data': '[SYSTEM] Manual reconnection failed - Arduino not found'})

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
        if not ser or not ser.is_open:
            print("⚠️ Arduino not connected - attempting reconnection before sending command")
            if not reconnect_arduino():
                socketio.emit('serial_log', {'data': '[ERROR] Cannot send command - Arduino not connected'})
                return
        try:
            serial_command = f"{command_to_send}\n"
            ser.write(serial_command.encode('utf-8'))
            log_message = f"[SENT] {serial_command.strip()}"
            print(log_message)
            socketio.emit('serial_log', {'data': log_message})
        except Exception as e:
            error_msg = f"[ERROR] Failed to send command: {e}"
            print(error_msg)
            socketio.emit('serial_log', {'data': error_msg})
            if "device not found" in str(e).lower() or "permission denied" in str(e).lower():
                print("🔄 Connection error during command send - attempting reconnection")
                reconnect_arduino()

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

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)
