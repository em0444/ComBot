"""combot controller."""
import math

from controller import wb as c_webots_api, \
    Motor  # The wb package gives you all the C-like methods, but the controller package wraps most of them in nicer-to-use classes.
from controllers.combot_controller.shared_dataclasses import Position
from fencing_actions import lunge, parry_high, parry_low, en_garde, enable_sensors, open_hand, close_hand
from ikpy_integration import initialise_ikpy_integration
from combot import Combot
import strategy as strat
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
    elif key_code == 79:  # O (Open Hand)
        open_hand()
    elif key_code == 67:  # C (Close Hand)
        close_hand()
    elif key_code == 65:  # A (Move Right Arm)
        initialise_ikpy_integration() # Initialize IKPY and move arm to opponent position
# combot.get_position()

timestep = int(combot.getBasicTimeStep())

wb.wb_keyboard_enable(timestep)

enable_sensors()

# Ensures sensors are enabled before reading them
for _ in range(5):
    combot.step(timestep)
print("Sensors ready.")

# en_garde()

# Main loop:
# - perform simulation steps until Webots is stopping the controller
done = False

combot.getDevice("wheel_left_joint").setVelocity(0.0)
combot.getDevice("wheel_right_joint").setVelocity(0.0)
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
    combot.update_internal_position_model()
    # combot.get_position()
    print(combot.get_position())
    if not done:
        print("sending command to move robot to position...")
        combot.move_to_position(Position(3, 1, math.pi))
        done = True

    # # enable RGBD camera
    # rgb_camera = wb.wb_robot_get_device("Astra rgb")
    # wb.wb_camera_enable(rgb_camera, timestep)
    # depth_camera = wb.wb_robot_get_device("Astra depth")
    # wb.wb_range_finder_enable(depth_camera, timestep)

    move = strat.strategy7(combot)
    if move is not None:
        move()

    pass

# Enter here exit cleanup code.
