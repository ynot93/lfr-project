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
    "A": GPIO.PWM(MOTOR_PINS["A"]["EN"], 100),
    "B": GPIO.PWM(MOTOR_PINS["B"]["EN"], 100),
}

motor_pwm["A"].start(50)
motor_pwm["B"].start(50)

# Ensure motors are stopped initially
for motor in MOTOR_PINS.values():
    GPIO.output(motor["IN1"], GPIO.LOW)
    GPIO.output(motor["IN2"], GPIO.LOW)

# Capture video feed
cap = cv2.VideoCapture(0)
cap.set(3, 160)  # Frame width
cap.set(4, 120)  # Frame height

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

        # Apply thresholding to detect the white line (masking tape)
        _, binary = cv2.threshold(roi, 200, 255, cv2.THRESH_BINARY)

        # Dilate the binary image to thicken the line for better detection
        kernel = np.ones((3, 3), np.uint8)  # Small kernel to keep the line sharp
        dilated = cv2.dilate(binary, kernel, iterations=1)

        # Find contours in the dilated image
        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Process the largest contour (assume it's the line)
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)

            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])  # Centroid x-coordinate
                frame_center = width // 2      # Frame center x-coordinate
                offset = cx - frame_center    # Offset from center

                print(f"CX: {cx}, Offset: {offset}")

                # Motor control logic based on offset
                if offset > 20:  # Line is to the right
                    print("Turn Left")
                    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.HIGH)
                    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)
                    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.LOW)
                    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.HIGH)

                elif -20 <= offset <= 20:  # Line is approximately centered
                    print("On Track")
                    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.HIGH)
                    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)
                    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.HIGH)
                    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)

                else:  # Line is to the left
                    print("Turn Right")
                    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.LOW)
                    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.HIGH)
                    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.HIGH)
                    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)
            else:
                print("No valid centroid found.")
        else:
            print("I don't see the line.")
            # Stop motors if no line is detected
            for motor in MOTOR_PINS.values():
                GPIO.output(motor["IN1"], GPIO.LOW)
                GPIO.output(motor["IN2"], GPIO.LOW)

        # Delay for stability
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Interrupted by user. Cleaning up...")
finally:
    # Cleanup GPIO and release resources
    for motor in MOTOR_PINS.values():
        GPIO.output(motor["IN1"], GPIO.LOW)
        GPIO.output(motor["IN2"], GPIO.LOW)
    motor_pwm["A"].stop()
    motor_pwm["B"].stop()
    GPIO.cleanup()
    cap.release()
