import pygame
import RPi.GPIO as GPIO
import time

# Initialize Pygame
pygame.init()

# Set window size (can be minimized)
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("Robot Keyboard Control")

# Define motor driver pins
MOTOR_PINS = {
    "A": {"EN": 18, "IN1": 23, "IN2": 24},  # Left motors
    "B": {"EN": 19, "IN1": 25, "IN2": 12},  # Right motors
}

# GPIO setup
GPIO.setmode(GPIO.BCM)
for motor in MOTOR_PINS.values():
    GPIO.setup(motor["EN"], GPIO.OUT)
    GPIO.setup(motor["IN1"], GPIO.OUT)
    GPIO.setup(motor["IN2"], GPIO.OUT)

# Initialize PWM for speed control
motor_pwm = {
    "A": GPIO.PWM(MOTOR_PINS["A"]["EN"], 100),  # Left motors
    "B": GPIO.PWM(MOTOR_PINS["B"]["EN"], 100),  # Right motors
}
motor_pwm["A"].start(0)
motor_pwm["B"].start(0)

# Stop all motors initially
def stop_motors():
    for motor in MOTOR_PINS.values():
        GPIO.output(motor["IN1"], GPIO.LOW)
        GPIO.output(motor["IN2"], GPIO.LOW)
    motor_pwm["A"].ChangeDutyCycle(0)
    motor_pwm["B"].ChangeDutyCycle(0)

stop_motors()

# Movement functions
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
    if pivot:
        motor_pwm["A"].ChangeDutyCycle(speed)
        motor_pwm["B"].ChangeDutyCycle(speed)
        GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.LOW)
        GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.HIGH)  # Reverse left motors
        GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)  # Forward right motors
    else:
        motor_pwm["A"].ChangeDutyCycle(speed * 0.5)
        motor_pwm["B"].ChangeDutyCycle(speed)
        GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)
        GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)

def turn_right(speed=50, pivot=False):
    if pivot:
        motor_pwm["A"].ChangeDutyCycle(speed)
        motor_pwm["B"].ChangeDutyCycle(speed)
        GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)  # Forward left motors
        GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.LOW)
        GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.HIGH)  # Reverse right motors
    else:
        motor_pwm["A"].ChangeDutyCycle(speed)
        motor_pwm["B"].ChangeDutyCycle(speed * 0.5)
        GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)
        GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)

# Game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Detect key press
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w or event.key == pygame.K_UP:  # Forward
                move_forward(70)
            elif event.key == pygame.K_s or event.key == pygame.K_DOWN:  # Backward
                move_backward(70)
            elif event.key == pygame.K_a or event.key == pygame.K_LEFT:  # Turn left
                turn_left(50, pivot=True)
            elif event.key == pygame.K_d or event.key == pygame.K_RIGHT:  # Turn right
                turn_right(50, pivot=True)

        # Detect key release (stop motors)
        elif event.type == pygame.KEYUP:
            stop_motors()

pygame.quit()
GPIO.cleanup()
