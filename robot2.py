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

        # Define the color range for masking
        low_b = np.uint8([5, 5, 5])
        high_b = np.uint8([0, 0, 0])
        mask = cv2.inRange(frame, high_b, low_b)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if contours:
            # Process the largest contour
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)

            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                print(f"CX: {cx}, CY: {cy}")

                # Motor control logic
                if cx >= 120:
                    print("Turn Left")
                    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.HIGH)
                    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)
                    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.LOW)
                    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.HIGH)

                elif 40 < cx < 120:
                    print("On Track")
                    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.HIGH)
                    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)
                    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.HIGH)
                    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)

                else:
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
