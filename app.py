from flask import Flask, render_template
from flask_socketio import SocketIO
import time
import random
import threading
from flask_cors import CORS # <-- 1. 이 줄을 추가하세요!

app = Flask(__name__)
CORS(app) # <-- 2. 이 줄을 추가하여 모든 출처에서의 요청을 허용합니다.
socketio = SocketIO(app)

# --- 시뮬레이션용 데이터 및 상태 변수 ---
current_depth = 0.0
current_pitch = 0.0
current_heading = 0
target_depth = 0.0
target_pitch = 0.0
glider_state = "IDLE" # IDLE, DIVING, ASCENDING, MOVING

# 백그라운드 스레드에서 실행될 센서 데이터 시뮬레이션 및 전송 함수
def background_thread():
    global current_depth, current_pitch, current_heading, glider_state

    while True:
        # 시뮬레이션 로직
        if glider_state == "DIVING":
            current_depth += random.uniform(0.1, 0.5)
            if current_depth >= target_depth:
                current_depth = target_depth
                glider_state = "IDLE"
        elif glider_state == "ASCENDING":
            current_depth -= random.uniform(0.1, 0.5)
            if current_depth <= 0.0:
                current_depth = 0.0
                glider_state = "IDLE"

        # 피치, 헤딩도 서서히 변경되도록 시뮬레이션 (간단화)
        current_pitch = round(random.uniform(-10.0, 10.0), 2)
        current_heading = random.randint(0, 359)

        # --- 센서 데이터 UI로 전송 ---
        sensor_data = {
            'depth': round(current_depth, 2),
            'pitch': current_pitch,
            'heading': current_heading,
            'temperature': round(random.uniform(20.0, 28.0), 2),
            'battery_voltage': round(random.uniform(10.5, 12.0), 2),
            'glider_state': glider_state
        }
        socketio.emit('sensor_update', sensor_data)
        time.sleep(1) # 1초마다 데이터 전송

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    print('Client connected!')
    # 백그라운드 스레드 시작 (한 번만 실행되도록)
    global thread
    if 'thread' not in globals() or not thread.is_alive():
        thread = threading.Thread(target=background_thread)
        thread.daemon = True # 메인 스레드 종료 시 함께 종료
        thread.start()

@socketio.on('control_event')
def handle_control_event(json_data):
    global target_depth, target_pitch, glider_state
    command = json_data.get('command')
    value = json_data.get('value') # 조종값 (예: 수심 목표값)

    print(f"Received command from UI: {command} (Value: {value})")

    # --- 글라이더 상태 및 목표값 업데이트 (시뮬레이션) ---
    if command == 'set_depth':
        try:
            depth_val = float(value)
            if depth_val >= 0:
                target_depth = depth_val
                if current_depth < target_depth:
                    glider_state = "DIVING"
                elif current_depth > target_depth:
                    glider_state = "ASCENDING" # 수심 감소 명령 시
                else:
                    glider_state = "IDLE"
                print(f"Target depth set to: {target_depth} m")
            else:
                print("Invalid depth value (must be >= 0)")
        except ValueError:
            print("Invalid depth value (not a number)")
    elif command == 'set_pitch':
        try:
            pitch_val = float(value)
            target_pitch = pitch_val
            print(f"Target pitch set to: {target_pitch} degrees")
            glider_state = "MOVING" # 피치 변경도 움직임으로 간주
        except ValueError:
            print("Invalid pitch value (not a number)")
    elif command == 'surface':
        target_depth = 0.0
        glider_state = "ASCENDING"
        print("Glider commanded to surface.")
    elif command == 'stop':
        glider_state = "IDLE"
        print("Glider commanded to stop.")

    # 여기에 아두이노로 명령을 보내는 로직이 들어갑니다.
    # 예: serial_port.write(f"{command},{value}\n".encode())

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)