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

Kp = 0.1  # Proportional constant for motor speed adjustment

# Inside the main loop
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Use a refined ROI
        height, width = gray.shape
        roi = gray[height // 2 :, :]  # Bottom half of the frame

        # Apply adaptive thresholding
        binary = cv2.adaptiveThreshold(
            roi, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2
        )
        kernel = np.ones((5, 5), np.uint8)
        dilated = cv2.dilate(binary, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Process the largest valid contour
            c = max((contour for contour in contours if cv2.contourArea(contour) > 500), key=cv2.contourArea, default=None)
            if c is not None:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    frame_center = width // 2
                    offset = cx - frame_center

                    # Proportional control
                    speed_adjust = int(Kp * abs(offset))
                    if offset > 20:  # Line is to the right
                        print("Turn Left")
                        motor_pwm["A"].ChangeDutyCycle(50 + speed_adjust)
                        motor_pwm["B"].ChangeDutyCycle(50 - speed_adjust)

                    elif -20 <= offset <= 20:  # Line is centered
                        print("On Track")
                        motor_pwm["A"].ChangeDutyCycle(50)
                        motor_pwm["B"].ChangeDutyCycle(50)

                    else:  # Line is to the left
                        print("Turn Right")
                        motor_pwm["A"].ChangeDutyCycle(50 - speed_adjust)
                        motor_pwm["B"].ChangeDutyCycle(50 + speed_adjust)
                else:
                    print("No valid centroid found.")
        else:
            print("I don't see the line.")
            for motor in MOTOR_PINS.values():
                GPIO.output(motor["IN1"], GPIO.LOW)
                GPIO.output(motor["IN2"], GPIO.LOW)

        time.sleep(0.03)

except KeyboardInterrupt:
    print("Interrupted by user. Cleaning up...")
finally:
    GPIO.cleanup()
    cap.release()

