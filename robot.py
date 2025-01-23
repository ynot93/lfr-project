import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
from flask import Flask, Response, jsonify, render_template
import threading
import eventlet

# Pin configuration
MOTOR_PINS = {
    "A": {"EN": 18, "IN1": 23, "IN2": 24},
    "B": {"EN": 19, "IN1": 25, "IN2": 12},
}

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for motor, pins in MOTOR_PINS.items():
    GPIO.setup(pins["EN"], GPIO.OUT)
    GPIO.setup(pins["IN1"], GPIO.OUT)
    GPIO.setup(pins["IN2"], GPIO.OUT)

# PWM setup
motor_pwm = {
    "A": GPIO.PWM(MOTOR_PINS["A"]["EN"], 100),
    "B": GPIO.PWM(MOTOR_PINS["B"]["EN"], 100),
}

for pwm in motor_pwm.values():
    pwm.start(0)

# Flask app setup
app = Flask(__name__)

# Shared variables
frame = None
processed_frame = None
metrics = {
    "motor_speeds": {"left": 0, "right": 0},
    "error": 0,
    "cx": 0,
    "fps": 0,
    "battery_voltage": 12.0
}
metrics_lock = threading.Lock()
frame_lock = threading.Lock()

@app.route('/metrics')
def get_metrics():
    with metrics_lock:
        return jsonify(metrics)

@app.route('/')
def dashboard():
    return render_template('dashboard.html')

@app.route('/video_feed')
def video_feed():
    def generate():
        global frame
        while True:
            with frame_lock:
                if frame is not None:
                    ret, jpeg = cv2.imencode('.jpg', frame)
                    if ret:
                        yield f"data:image/jpeg;base64,{base64.b64encode(jpeg.tobytes()).decode('utf-8')}\n\n"
            eventlet.sleep(0.1)  # Introduce a small delay for smoother streaming
    return Response(generate(), mimetype='text/event-stream')

@app.route('/processed_feed')
def processed_feed():
    def generate():
        global processed_frame
        while True:
            with frame_lock:
                if processed_frame is not None:
                    ret, jpeg = cv2.imencode('.jpg', processed_frame)
                    if ret:
                        yield f"data:image/jpeg;base64,{base64.b64encode(jpeg.tobytes()).decode('utf-8')}\n\n"
            eventlet.sleep(0.1)  # Introduce a small delay for smoother streaming
    return Response(generate(), mimetype='text/event-stream')

def set_motor_speed(motor, speed):
    if speed > 0:
        GPIO.output(MOTOR_PINS[motor]["IN1"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS[motor]["IN2"], GPIO.LOW)
    elif speed < 0:
        GPIO.output(MOTOR_PINS[motor]["IN1"], GPIO.LOW)
        GPIO.output(MOTOR_PINS[motor]["IN2"], GPIO.HIGH)
    else:
        GPIO.output(MOTOR_PINS[motor]["IN1"], GPIO.LOW)
        GPIO.output(MOTOR_PINS[motor]["IN2"], GPIO.LOW)

    motor_pwm[motor].ChangeDutyCycle(abs(speed))

def cleanup():
    for pwm in motor_pwm.values():
        pwm.stop()
    GPIO.cleanup()

def process_frame(frame):
    global processed_frame

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)
    height, width = binary.shape
    roi = binary[int(height / 2):, :]
    M = cv2.moments(roi)
    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"])
    else:
        cx = width // 2

    # Draw feedback on the processed frame
    processed_frame = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    cv2.line(processed_frame, (cx, 0), (cx, height), (0, 255, 0), 2)
    cv2.line(processed_frame, (width // 2, 0), (width // 2, height), (255, 0, 0), 2)
    
    return cx, width

def control_motors(error):
    base_speed = 50
    k_p = 0.1
    adjustment = k_p * error
    left_speed = base_speed - adjustment
    right_speed = base_speed + adjustment
    left_speed = max(min(left_speed, 100), -100)
    right_speed = max(min(right_speed, 100), -100)
    set_motor_speed("A", left_speed)
    set_motor_speed("B", right_speed)

    return left_speed, right_speed

def main():
    global frame
    try:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error: Could not open camera.")
            return

        flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False))
        flask_thread.daemon = True
        flask_thread.start()

        # Track FPS
        fps_start_time = time.time()
        frame_count = 0

        while True:
            ret, new_frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            cx, width = process_frame(new_frame)
            error = cx - (width // 2)
            left_speed, right_speed = control_motors(error)  # Capture motor speeds

            # Update shared metrics dictionary
            with metrics_lock:
                metrics["motor_speeds"]["left"] = left_speed
                metrics["motor_speeds"]["right"] = right_speed
                metrics["error"] = error
                metrics["cx"] = cx

                # Calculate FPS
                frame_count += 1
                elapsed_time = time.time() - fps_start_time
                if elapsed_time > 0:
                    metrics["fps"] = frame_count / elapsed_time

                # Dummy voltage drop simulation
                metrics["battery_voltage"] = max(12.0 - (frame_count * 0.001), 6.0)  # Simulated voltage drop

            # Update the shared frame for live feed
            with frame_lock:
                frame = new_frame

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        cap.release()
        cleanup()

if __name__ == "__main__":
    eventlet.wsgi.server(eventlet.listen(('', 5000)), app) 
