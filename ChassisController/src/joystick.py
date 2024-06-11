import pygame
import serial
import time

# Initialize Pygame for controller inputs
pygame.init()
pygame.joystick.init()

# Check for joystick
if pygame.joystick.get_count() == 0:
    raise Exception("No joystick detected")
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Setup Serial connection (update 'COM3' to your actual serial port)
ser = serial.Serial('COM6', 115200)

def get_joystick_values():
    pygame.event.pump()
    # Assuming axis 0 for horizontal (left -1 to right +1) and axis 1 for vertical (up -1 to down +1)
    horizontal = joystick.get_axis(0)
    vertical = -1 * joystick.get_axis(1)
    return horizontal, vertical

def check_bumpers():
    # Assuming buttons 4 and 5 are bumpers (adjust if needed)
    left_bumper = joystick.get_button(1)
    right_bumper = joystick.get_button(4)
    return left_bumper, right_bumper


def send_serial_commands(horizontal, vertical, b, right_bumper):
    # Convert joystick values to your command format, here's a simple example
    if int(b > 0):
        command = "HALT\n"
    elif int(right_bumper) > 0:
        command = "RESET\n"
    else:
        command = "JOY V:{:0.3f} H:{:0.3f}\n".format(vertical, horizontal)

    ser.write(command.encode())
    print(command)


try:
    while True:
        h, v = get_joystick_values()
        lb, rb = check_bumpers()
        send_serial_commands(h, v, lb, rb)
        time.sleep(0.1)  # Adjust delay to your needs

except KeyboardInterrupt:
    print("Program exited by user")
finally:
    pygame.quit()
    ser.close()

