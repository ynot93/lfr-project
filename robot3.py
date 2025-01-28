import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

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

def move_backward(speed=50):
    motor_pwm["A"].ChangeDutyCycle(speed)
    motor_pwm["B"].ChangeDutyCycle(speed)
    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.HIGH)

def turn_left(speed=50, pivot=False):
    if pivot:  # Sharp pivot turn
        motor_pwm["A"].ChangeDutyCycle(speed)
        motor_pwm["B"].ChangeDutyCycle(speed)
        GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.LOW)
        GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.HIGH)  # Reverse left motor
        GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)  # Forward right motor
    else:  # Smooth left turn
        motor_pwm["A"].ChangeDutyCycle(speed * 0.5)  # Slow left motor
        motor_pwm["B"].ChangeDutyCycle(speed)  # Full speed right motor
        GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)
        GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)

def turn_right(speed=50, pivot=False):
    if pivot:  # Sharp pivot turn
        motor_pwm["A"].ChangeDutyCycle(speed)
        motor_pwm["B"].ChangeDutyCycle(speed)
        GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)  # Forward left motor
        GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.LOW)
        GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.HIGH)  # Reverse right motor
    else:  # Smooth right turn
        motor_pwm["A"].ChangeDutyCycle(speed)  # Full speed left motor
        motor_pwm["B"].ChangeDutyCycle(speed * 0.5)  # Slow right motor
        GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)
        GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)

# Capture video feed
cap = cv2.VideoCapture(0)
time.sleep(1)
cap.set(3, 320)  # Frame width
cap.set(4, 240)  # Frame height

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame.")
            break

        # Convert frame to grayscale for faster processing
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Define a region of interest (ROI) in the lower part of the frame
        height, width = gray.shape
        roi = gray[height // 2 :, :]  # Only process the bottom half of the frame

        # Apply adaptive thresholding to detect the white line
        binary = cv2.adaptiveThreshold(roi, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

        # Dilate the binary image to thicken the line for better detection
        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(binary, kernel, iterations=1)

        # Find contours in the dilated image
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)  # Use the largest contour
            M = cv2.moments(largest_contour)

            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])  # Centroid x-coordinate
                frame_center = width // 2  # Frame center x-coordinate
                offset = cx - frame_center  # Offset from center

                print(f"CX: {cx}, Offset: {offset}")

                # Motor control logic based on offset
                if offset > 20:  # Line is to the right
                    turn_left(speed=50)
                elif -20 <= offset <= 20:  # Line is approximately centered
                    move_forward(speed=50)
                else:  # Line is to the left
                    turn_right(speed=50)
            else:
                print("No valid centroid found.")
                stop_motors()
        else:
            print("I don't see the line.")
            stop_motors()

        # Delay for stability
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Interrupted by user. Cleaning up...")
finally:
    # Cleanup GPIO and release resources
    stop_motors()
    motor_pwm["A"].stop()
    motor_pwm["B"].stop()
    GPIO.cleanup()
    cap.release()
