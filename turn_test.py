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

def stop_motors():
    for motor in MOTOR_PINS.values():
        GPIO.output(motor["IN1"], GPIO.LOW)
        GPIO.output(motor["IN2"], GPIO.LOW)
    motor_pwm["A"].ChangeDutyCycle(0)
    motor_pwm["B"].ChangeDutyCycle(0)

def turn_left(speed=80, duration=1):
    print("Turning Left")
    motor_pwm["A"].ChangeDutyCycle(speed)  # Stop left motor
    motor_pwm["B"].ChangeDutyCycle(speed)  # Run right motor
    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.HIGH)  # Left motor off
    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)  # Right motor forward
    time.sleep(duration)
    stop_motors()

def turn_right(speed=80, duration=1):
    print("Turning Right")
    motor_pwm["A"].ChangeDutyCycle(speed)  # Run left motor
    motor_pwm["B"].ChangeDutyCycle(speed)  # Stop right motor
    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)  # Left motor forward
    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.HIGH)  # Right motor backward
    time.sleep(duration)
    stop_motors()

try:
    while True:
        user_input = input("Enter 'l' for left turn, 'r' for right turn, 'q' to quit: ").strip().lower()
        if user_input == 'l':
            turn_left(speed=100, duration=1)
        elif user_input == 'r':
            turn_right(speed=100, duration=1)
        elif user_input == 'q':
            break
        else:
            print("Invalid input! Enter 'l' or 'r'.")

except KeyboardInterrupt:
    print("Interrupted by user. Cleaning up...")
finally:
    stop_motors()
    motor_pwm["A"].stop()
    motor_pwm["B"].stop()
    GPIO.cleanup()
