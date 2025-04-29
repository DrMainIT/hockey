import RPi.GPIO as GPIO
import time

# Pin setup
DIR_PINS = [23, 22]
STEP_PINS = [24, 27]
ENABLE_PINS = [4, 5]
MICROSTEP_PINS = [26, 6]
STEPS_PR = 2000  # Steps per revolution

# Setup GPIO
GPIO.setmode(GPIO.BCM)
for pin in DIR_PINS + STEP_PINS + ENABLE_PINS + MICROSTEP_PINS:
    GPIO.setup(pin, GPIO.OUT)

# Optional: Set microstepping mode
for pin in MICROSTEP_PINS:
    GPIO.output(pin, GPIO.HIGH)  # Adjust based on your driver

# Function to step both motors simultaneously
def step_motors(direction_0, direction_1):
    GPIO.output(DIR_PINS[0], direction_0)
    GPIO.output(DIR_PINS[1], direction_1)

    # Enable both motors
    GPIO.output(ENABLE_PINS[0], GPIO.LOW)
    GPIO.output(ENABLE_PINS[1], GPIO.LOW)

    for _ in range(STEPS_PR):
        # Step both motors at the same time
        GPIO.output(STEP_PINS[0], GPIO.HIGH)
        GPIO.output(STEP_PINS[1], GPIO.HIGH)
        time.sleep(0.0005)
        GPIO.output(STEP_PINS[0], GPIO.LOW)
        GPIO.output(STEP_PINS[1], GPIO.LOW)
        time.sleep(0.0005)

    # Optionally disable motors
    GPIO.output(ENABLE_PINS[0], GPIO.HIGH)
    GPIO.output(ENABLE_PINS[1], GPIO.HIGH)

try:
    while True:
        step_motors(GPIO.HIGH, GPIO.HIGH)  # Both forward
        time.sleep(1)
        step_motors(GPIO.LOW, GPIO.LOW)    # Both backward
        time.sleep(1)

except KeyboardInterrupt:
    print("Exiting...")

finally:
    GPIO.cleanup()
