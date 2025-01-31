from flask import Flask, render_template, Response, jsonify
import cv2
import threading
import time
import RPi.GPIO as GPIO
import numpy as np

app = Flask(__name__)

cx_position = 0
offset_value = 0
direction = "Stopped"
rfid_tag = "None"

cap = cv2.VideoCapture(0)
cap.set(3, 320)
cap.set(4, 240)

# Pin configuration
MOTOR_PINS = {
    "A": {"EN": 18, "IN1": 23, "IN2": 24},
    "B": {"EN": 19, "IN1": 25, "IN2": 12},
}

# GPIO setup
GPIO.setmode(GPIO.BCM)
for motor in MOTOR_PINS.values():
    GPIO.setup(motor["EN"], GPIO.OUT)
    GPIO.setup(motor["IN1"], GPIO.OUT)
    GPIO.setup(motor["IN2"], GPIO.OUT)

# Initialize PWM for motors
motor_pwm = {
    "A": GPIO.PWM(MOTOR_PINS["A"]["EN"], 100),  # Left motor
    "B": GPIO.PWM(MOTOR_PINS["B"]["EN"], 100),  # Right motor
}
motor_pwm["A"].start(0)
motor_pwm["B"].start(0)

# RFID Reader Pins (Wiegand D0 & D1)
D0_PIN = 17  # Green wire
D1_PIN = 27  # White wire

# Setup GPIO for Wiegand
GPIO.setmode(GPIO.BCM)
GPIO.setup(D0_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(D1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Variables for RFID decoding
rfid_bits = []
last_read_time = time.time()

def wiegand_callback(channel):
    """Reads data from the Wiegand RFID reader."""
    global rfid_bits, last_read_time, rfid_tag
    bit = 0 if channel == D0_PIN else 1
    rfid_bits.append(bit)
    last_read_time = time.time()

    if len(rfid_bits) >= 26:  # Wiegand 26-bit format
        card_id = int("".join(str(b) for b in rfid_bits[1:-1]), 2)  # Remove parity bits
        rfid_tag = str(card_id)
        rfid_bits.clear()

# Attach interrupts to detect RFID signals
GPIO.add_event_detect(D0_PIN, GPIO.FALLING, callback=wiegand_callback)
GPIO.add_event_detect(D1_PIN, GPIO.FALLING, callback=wiegand_callback)

# Ensure motors are stopped initially
def stop_motors():
    for motor in MOTOR_PINS.values():
        GPIO.output(motor["IN1"], GPIO.LOW)
        GPIO.output(motor["IN2"], GPIO.LOW)
    motor_pwm["A"].ChangeDutyCycle(0)
    motor_pwm["B"].ChangeDutyCycle(0)

stop_motors()

# Motor control functions
def move_forward(speed=50):
    motor_pwm["A"].ChangeDutyCycle(speed)
    motor_pwm["B"].ChangeDutyCycle(speed)
    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)

def turn_left(speed=50):
    motor_pwm["A"].ChangeDutyCycle(speed * 0.5)  # Slow left motor
    motor_pwm["B"].ChangeDutyCycle(speed)  # Full speed right motor
    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)

def turn_right(speed=50):
    motor_pwm["A"].ChangeDutyCycle(speed)  # Full speed left motor
    motor_pwm["B"].ChangeDutyCycle(speed * 0.5)  # Slow right motor
    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)

def process_video():
    global cx_position, offset_value, direction
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # Convert frame to grayscale for faster processing
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Define a region of interest (ROI) in the lower part of the frame
        height, width = gray.shape
        roi = gray[height // 2 :, :]  # Only process the bottom half of the frame

        # Apply adaptive thresholding to detect the white line
        _, binary = cv2.threshold(roi, 150, 255, cv2.THRESH_BINARY)

        # Dilate the binary image to thicken the line for better detection
        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(binary, kernel, iterations=1)

        # Find contours in the binary image
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            min_contour_area = 500
            filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]
            
            if filtered_contours:
                largest_contour = max(contours, key=cv2.contourArea)  # Use the largest contour
                M = cv2.moments(largest_contour)

                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])  # Centroid x-coordinate
                    frame_center = width // 2  # Frame center x-coordinate
                    offset = cx - frame_center  # Offset from center

                    print(f"CX: {cx}, Offset: {offset}")

                    # Draw contours on the original frame
                    cv2.drawContours(frame, [largest_contour + [0, height // 2]], -1, (0, 255, 0), 2)
                    cv2.circle(frame, (cx, height // 2), 5, (0, 0, 255), -1)  # Draw centroid

                    # Motor control logic based on offset
                    if offset > 20:  # Line is to the right
                        direction = "Turning Left"
                        turn_left(speed=50)
                    elif -20 <= offset <= 20:  # Line is approximately centered
                        direction = "Moving Forward"
                        move_forward(speed=50)
                    else:  # Line is to the left
                        direction = "Turning Right"
                        turn_right(speed=50)
                else:
                    print("No valid centroid found.")
                    direction = "No valid centroid found."
                    stop_motors()
            else:
                print("No valid lane detected.")
                direction = "No valid lane detected."
                stop_motors()
        else:
            print("No contours detected.")
            direction = "No contours detected."
            stop_motors()

def generate_frames():
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        _, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def dashboard():
    return render_template("dash-board.html")

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/data')
def get_data():
    return jsonify(cx=cx_position, offset=offset_value, direction=direction)

if __name__ == "__main__":
    threading.Thread(target=process_video, daemon=True).start()
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
