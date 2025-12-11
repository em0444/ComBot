"""player controller."""
import math
from typing import Callable, Dict

from controller import wb as c_webots_api, \
    Motor  # The wb package gives you all the C-like methods, but the controller package wraps most of them in nicer-to-use classes.
from combot import Combot
from shared_dataclasses import Position
from arm import Arm
import fencing_constants as fc
import fencing_actions as fence
import strategy as strat
import csv

# Movement Keyboard codes (Webots specific)
KEY_UP = 315
KEY_DOWN = 317
KEY_RIGHT = 316
KEY_LEFT = 314

# Fencing Action Keys
KEY_LUNGE = 32      # Spacebar
KEY_PARRY_HIGH = 81 # Q
KEY_PARRY_LOW = 90  # Z
KEY_EN_GARDE = 82   # R

# Shorthand alias for the Webots API module
wb = c_webots_api.wb

def handle_movement_speed(key, max_speed):
    speed_left = 0.0
    speed_right = 0.0

    if key == KEY_UP: 
        speed_left, speed_right = max_speed, max_speed
    elif key == KEY_DOWN: 
        speed_left, speed_right = -max_speed, -max_speed
    elif key == KEY_RIGHT:  
        speed_left, speed_right = max_speed, -max_speed
    elif key == KEY_LEFT:  
        speed_left, speed_right = -max_speed, max_speed     
    
    return speed_left, speed_right
            
def handle_fencing_action(key: int, arm: Arm):
    # Map keys to their corresponding fencing action functions
    action_map: Dict[int, Callable] = {
        KEY_LUNGE:      fence.lunge,
        KEY_PARRY_HIGH: fence.parry_high,
        KEY_PARRY_LOW:  fence.parry_low,
        KEY_EN_GARDE:   fence.en_garde
    }
    # Execute if key exists in map
    if key in action_map:
        action_map[key]()

def main():
    combot: Combot = Combot()
    timestep = int(combot.getBasicTimeStep())
    arm: Arm = Arm(combot, fc.RIGHT_ARM_CONFIG)
    fence.init(arm) # Initialise fencing module with arm reference
    
    wb.wb_keyboard_enable(timestep)

    # Get references to the wheel motors
    left_wheel = combot.getDevice("wheel_left_joint")
    right_wheel = combot.getDevice("wheel_right_joint")

    try:
        while combot.step(timestep) != -1:

            key = wb.wb_keyboard_get_key()

            if key > 0:
                max_speed = left_wheel.getMaxVelocity()
                speed_left, speed_right = handle_movement_speed(key, max_speed)

                left_wheel.setVelocity(speed_left)
                right_wheel.setVelocity(speed_right)

                handle_fencing_action(key, arm)

            fence.check_hit()
            
    except KeyboardInterrupt:   
        print("Controller stopped by user.")
        pass

if __name__ == "__main__":
    main()
    