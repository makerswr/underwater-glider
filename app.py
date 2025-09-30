from flask import Flask, render_template, Response
from flask_socketio import SocketIO
from flask_cors import CORS
import cv2
import time
import threading
import serial
import json

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app)

# --- 아두이노 연결 ---
SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 115200
ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"✅ Connected to Arduino on {SERIAL_PORT}")
except serial.SerialException:
    print(f"❌ Failed to connect to Arduino.")

# --- 카메라 초기화 ---
camera = cv2.VideoCapture(0)
if camera.isOpened():
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("✅ Camera initialized.")
else:
    print("❌ Failed to open camera.")

# --- 백그라운드 스레드 (아두이노 데이터 수신) ---
def arduino_reader_thread():
    while True:
        if ser and ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    socketio.emit('serial_log', {'data': f'[RECV] {line}'})
                    if line.startswith('{') and line.endswith('}'):
                        sensor_data = json.loads(line)
                        socketio.emit('sensor_update', sensor_data)
            except Exception as e:
                socketio.emit('serial_log', {'data': f"[ERROR] {e}"})
        time.sleep(0.1)

# --- 웹 서버 로직 ---
def generate_frames():
    while True:
        if camera.isOpened():
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
    global thread
    if 'thread' not in globals() or not thread.is_alive():
        thread = threading.Thread(target=arduino_reader_thread)
        thread.daemon = True
        thread.start()

@socketio.on('control_event')
def handle_control_event(json_data):
    command_to_send = json_data.get('command')
    if ser and command_to_send:
        serial_command = f"{command_to_send}\n"
        ser.write(serial_command.encode('utf-8'))
        log_message = f"[SENT] {serial_command.strip()}"
        print(log_message)
        socketio.emit('serial_log', {'data': log_message})

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)