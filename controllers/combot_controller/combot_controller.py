"""combot controller."""

from controller import wb as c_webots_api, \
    Motor  # The wb package gives you all the C-like methods, but the controller package wraps most of them in nicer-to-use classes.
from fencing_actions import lunge, parry_high, parry_low, en_garde, move_to_pose
from combot import Combot
from strategy import decideMove
decideMove = lambda : None
wb = c_webots_api.wb

combot: Combot = Combot()

def check_keyboard(key):
    max_speed = combot.getDevice("wheel_left_joint").getMaxVelocity()
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
            
def check_manual_fencing_action(key_code):

    if key_code == 32:  # Spacebar (Lunge)
        lunge()
    elif key_code == 81:  # Q (Parry High)
        parry_high() 
    elif key_code == 90:  # Z (Parry Low)
        parry_low()
    elif key_code == 82:  # R (En Guard)
        en_garde()

# combot.get_position()

timestep = int(combot.getBasicTimeStep())

wb.wb_keyboard_enable(timestep)

initialise_motors()
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while combot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    key = wb.wb_keyboard_get_key()

    check_keyboard(key)
    check_manual_fencing_action(key)

    move = decideMove()
    if move is not None:
        move()

    pass

# Enter here exit cleanup code.
