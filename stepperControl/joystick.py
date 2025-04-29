import pygame
import time
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

# Initialize Pygame and the joystick
pygame.init()
pygame.joystick.init()


# Stepper driver parameters
MAX_SPEED_HZ = 1000  # Max steps per second
MIN_DELAY = 1.0 / MAX_SPEED_HZ / 4

def get_delay_from_input(value):
    """Convert joystick input (-1 to 1) to delay time between steps."""
    speed = abs(value)
    if speed < 0.05:
        return None  # Dead zone
    delay = 1.0 / (MAX_SPEED_HZ * speed) / 4  # Half-step delay
    return delay


# Check if any joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick detected.")
    exit()

# Use the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Detected joystick: {joystick.get_name()}")



# Read input in a loop
try:
    while True:
        pygame.event.pump()  # Process event queue

        # Example: Read left analog stick (axis 0 and 1)
        left_x = joystick.get_axis(0)
        left_y = joystick.get_axis(1)
        delay = get_delay_from_input(left_x)
        if delay is not None:
            # Set direction
            GPIO.output(DIR_PINS[0], GPIO.HIGH if left_x > 0 else GPIO.LOW)
            # Step
            GPIO.output(STEP_PINS[0], GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(STEP_PINS[0], GPIO.LOW)
            time.sleep(delay)
        else:
            time.sleep(0.01)  # Idle wait

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    joystick.quit()
    pygame.quit()