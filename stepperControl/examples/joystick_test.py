import pygame
import time

# Initialize Pygame and the joystick
pygame.init()
pygame.joystick.init()

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

        # Example: Read Cross (X) button (usually button 0)
        x_button = joystick.get_button(0)

        print(f"Left Stick X: {left_x:.2f}, Y: {left_y:.2f}, X Button: {x_button}")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    joystick.quit()
    pygame.quit()