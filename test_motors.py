import RPi.GPIO as GPIO
import time

# Motor pin configuration
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
pwm_a = GPIO.PWM(MOTOR_PINS["A"]["EN"], 100)
pwm_b = GPIO.PWM(MOTOR_PINS["B"]["EN"], 100)

# Start PWM
pwm_a.start(50)  # Set initial duty cycle (speed)
pwm_b.start(50)

try:
    # Test left motor forward
    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)
    print("Left motor forward")
    time.sleep(2)

    # Test left motor reverse
    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.HIGH)
    print("Left motor reverse")
    time.sleep(2)

    # Stop left motor
    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)

    # Test right motor forward
    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)
    print("Right motor forward")
    time.sleep(2)

    # Test right motor reverse
    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.HIGH)
    print("Right motor reverse")
    time.sleep(2)

    # Stop right motor
    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Stop PWM and cleanup
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
    print("GPIO cleanup complete")
