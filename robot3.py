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
        _, binary = cv2.threshold(roi, 150, 255, cv2.THRESH_BINARY)

        # Dilate the binary image to thicken the line for better detection
        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(binary, kernel, iterations=1)

        # Find contours in the binary image
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            min_contoured_area = 500
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
                        turn_left(speed=50)
                    elif -20 <= offset <= 20:  # Line is approximately centered
                        move_forward(speed=50)
                    else:  # Line is to the left
                        turn_right(speed=50)
                else:
                    print("No valid centroid found.")
                    stop_motors()
            else:
                print("No valid lane detected.")
                stop_motors()
        else:
            print("No contours detected.")
            stop_motors()

        # Show video feed
        cv2.imshow("Original Frame", frame)
        cv2.imshow("Binary Image", binary)

        # Stop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

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
    cv2.destroyAllWindows()

