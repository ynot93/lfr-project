import RPi.GPIO as GPIO
import time

# GPIO configuration
MOTOR_PINS = {
    "A": {"EN": 18, "IN1": 23, "IN2": 24},  # Left Motor
    "B": {"EN": 19, "IN1": 25, "IN2": 12},  # Right Motor
}

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    for motor in MOTOR_PINS.values():
        GPIO.setup(motor["EN"], GPIO.OUT)
        GPIO.setup(motor["IN1"], GPIO.OUT)
        GPIO.setup(motor["IN2"], GPIO.OUT)
    
    # Set up PWM for speed control
    pwm_a = GPIO.PWM(MOTOR_PINS["A"]["EN"], 100)  # 100 Hz PWM frequency
    pwm_b = GPIO.PWM(MOTOR_PINS["B"]["EN"], 100)
    
    pwm_a.start(80)  # 50% duty cycle
    pwm_b.start(80)
    return pwm_a, pwm_b

def move_forward():
    print("Moving Forward")
    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)

def move_backward():
    print("Moving Backward")
    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.HIGH)

def turn_left():
    print("Turning Left")
    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.LOW)   # Stop left motor
    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.HIGH)  # Right motor moves forward
    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)

def turn_right():
    print("Turning Right")
    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.HIGH)  # Left motor moves forward
    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.LOW)   # Stop right motor
    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)

def stop():
    print("Stopping")
    GPIO.output(MOTOR_PINS["A"]["IN1"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["A"]["IN2"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["B"]["IN1"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["B"]["IN2"], GPIO.LOW)

def cleanup(pwm_a, pwm_b):
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()

if __name__ == "__main__":
    try:
        pwm_a, pwm_b = setup_gpio()
        
        while True:
            # Test forward movement
            move_forward()
            time.sleep(3)  # Move forward for 3 seconds
            
            stop()
            time.sleep(1)  # Pause for 1 second
            
            # Test backward movement
            move_backward()
            time.sleep(3)  # Move backward for 3 seconds
            
            stop()
            time.sleep(1)  # Pause for 1 second
            
            # Test left turn
            turn_left()
            time.sleep(2)  # Turn left for 2 seconds
            
            stop()
            time.sleep(1)  # Pause for 1 second
            
            # Test right turn
            turn_right()
            time.sleep(2)  # Turn right for 2 seconds
            
            stop()
            time.sleep(1)  # Pause for 1 second
            
    except KeyboardInterrupt:
        print("Test interrupted by user")
    finally:
        cleanup(pwm_a, pwm_b)
