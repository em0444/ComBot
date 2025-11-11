"""tiagopp controller."""

from controller import wb as c_webots_api, \
    Motor  # The wb package gives you all the C-like methods, but the controller package wraps most of them in nicer-to-use classes.
from initialisation import initialise_motors
from combot import Combot

wb = c_webots_api.wb

combot: Combot = Combot()

def check_keyboard():
    max_speed = combot.getDevice("wheel_left_joint").getMaxVelocity()
    key = wb.wb_keyboard_get_key()
    speed_left, speed_right = 0, 0

    if key == 315: #up
        speed_left = max_speed
        speed_right = max_speed
    elif key == 317: #down
        speed_left = -max_speed
        speed_right = -max_speed
    elif key == 316:  # right
        speed_left = max_speed
        speed_right = -max_speed
    elif key == 314:  # left
        speed_left = -max_speed
        speed_right = max_speed

    if key > 0:
        combot.getDevice("wheel_left_joint").setPosition(float('inf'))
        combot.getDevice("wheel_right_joint").setPosition(float('inf'))
        combot.getDevice("wheel_left_joint").setVelocity(speed_left)
        combot.getDevice("wheel_right_joint").setVelocity(speed_right)

# combot.get_position()

timestep = int(combot.getBasicTimeStep())

initialise_motors()

wb.wb_keyboard_enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while combot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

    check_keyboard()

    pass

# Enter here exit cleanup code.
