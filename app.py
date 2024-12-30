import cv2
import numpy as np
import RPi.GPIO as GPIO
from flask import Flask, render_template, Response, jsonify
from flask_socketio import SocketIO
import time
import keyboard

# Flask and SocketIO setup
app = Flask(__name__)
socketio = SocketIO(app)

# GPIO Pin Setup
MOTOR_PINS = {
    "A": {"EN": 18, "IN1": 23, "IN2": 24},
    "B": {"EN": 19, "IN1": 25, "IN2": 12}
}

GPIO.setmode(GPIO.BCM)
for motor, pins in MOTOR_PINS.items():
    GPIO.setup(pins["EN"], GPIO.OUT)
    GPIO.setup(pins["IN1"], GPIO.OUT)
    GPIO.setup(pins["IN2"], GPIO.OUT)

motor_pwm = {
    "A": GPIO.PWM(MOTOR_PINS["A"]["EN"], 100),
    "B": GPIO.PWM(MOTOR_PINS["B"]["EN"], 100)
}
motor_pwm["A"].start(0)
motor_pwm["B"].start(0)

# Motor Control Functions
def set_motor(motor, direction, speed):
    if direction == "forward":
        GPIO.output(MOTOR_PINS[motor]["IN1"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS[motor]["IN2"], GPIO.LOW)
    elif direction == "backward":
        GPIO.output(MOTOR_PINS[motor]["IN1"], GPIO.LOW)
        GPIO.output(MOTOR_PINS[motor]["IN2"], GPIO.HIGH)
    else:
        GPIO.output(MOTOR_PINS[motor]["IN1"], GPIO.LOW)
        GPIO.output(MOTOR_PINS[motor]["IN2"], GPIO.LOW)
    motor_pwm[motor].ChangeDutyCycle(speed)

def stop_motors():
    set_motor("A", "stop", 0)
    set_motor("B", "stop", 0)

# Camera and Lane Detection
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Lower resolution for better performance
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

frame_width = 320
kp = 0.1  # PID constant
threshold = frame_width // 10  # Center alignment threshold

# Global Variables
data = {"cx": None, "error": None, "fps": 0, "left_speed": 0, "right_speed": 0}
fallback_timer = None  # Timer for fallback mechanism
manual_mode = False  # Manual vs. Automatic mode

# Line Detection
def find_line_center(mask):
    M = cv2.moments(mask)
    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"])
        return cx
    return None

def process_frame():
    global data, fallback_timer
    while True:
        if manual_mode:
            time.sleep(0.1)  # Skip processing in manual mode
            continue

        ret, frame = camera.read()
        if not ret:
            continue

        resized_frame = cv2.resize(frame, (320, 240))
        gray = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY_INV)
        roi = thresh[120:, :]  # Region of Interest (lower half)
        cx = find_line_center(roi)

        if cx is not None:
            error = cx - (frame_width // 2)
            speed_adjustment = kp * error

            left_speed = max(0, 60 - speed_adjustment)
            right_speed = max(0, 60 + speed_adjustment)

            set_motor("A", "forward", left_speed)
            set_motor("B", "forward", right_speed)

            # Reset fallback timer
            fallback_timer = time.time()

            # Update data
            data["cx"] = cx
            data["error"] = error
            data["left_speed"] = left_speed
            data["right_speed"] = right_speed
        else:
            # Check fallback timeout
            if fallback_timer and time.time() - fallback_timer > 2:  # 2 seconds timeout
                stop_motors()
                data["cx"] = None
                data["error"] = None
                data["left_speed"] = 0
                data["right_speed"] = 0


def manual_control():
    global manual_mode

    print("Press 'm' to toggle manual mode.")
    print("In manual mode, use 'w', 's', 'a', 'd' to control the robot and 'q' to quit.")

    while True:
        # Toggle manual mode
        if keyboard.is_pressed('m'):
            manual_mode = not manual_mode
            print(f"Manual mode {'enabled' if manual_mode else 'disabled'}")
            if not manual_mode:
                stop_motors()
            keyboard.wait('m')  # Prevent repeated toggling until the key is released

        if manual_mode:
            # Forward
            if keyboard.is_pressed('w'):
                set_motor("A", "forward", 60)
                set_motor("B", "forward", 60)

            # Backward
            elif keyboard.is_pressed('s'):
                set_motor("A", "backward", 60)
                set_motor("B", "backward", 60)

            # Left
            elif keyboard.is_pressed('a'):
                set_motor("A", "forward", 30)
                set_motor("B", "forward", 60)

            # Right
            elif keyboard.is_pressed('d'):
                set_motor("A", "forward", 60)
                set_motor("B", "forward", 30)

            # Stop the robot when no movement keys are pressed
            elif not (keyboard.is_pressed('w') or keyboard.is_pressed('s') or keyboard.is_pressed('a') or keyboard.is_pressed('d')):
                stop_motors()

        # Quit the manual control loop
        if keyboard.is_pressed('q'):
            print("Exiting manual control...")
            stop_motors()
            break

# Flask Routes
@app.route('/')
def index():
    return render_template('dashboard.html')

@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            frame = data.get("frame", b'')
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/robot_data')
def robot_data():
    return jsonify(data)

# Run Flask App
if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)

