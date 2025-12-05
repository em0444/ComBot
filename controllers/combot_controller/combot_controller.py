"""combot controller."""
import math
from typing import Callable, Dict

from controller import wb as c_webots_api, \
    Motor  # The wb package gives you all the C-like methods, but the controller package wraps most of them in nicer-to-use classes.
from combot import Combot
from shared_dataclasses import Position
from arm_controller import ArmController
import fencing_constants as fc
import fencing_actions as fence
import strategy as strat

# Keyboard codes (Webots specific)
KEY_UP = 315
KEY_DOWN = 317
KEY_RIGHT = 316
KEY_LEFT = 314

# Fencing Action Keys
KEY_LUNGE = 32      # Spacebar
KEY_PARRY_HIGH = 81 # Q
KEY_PARRY_LOW = 90  # Z
KEY_EN_GARDE = 82   # R
KEY_MOVE_ARM = 65   # A

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
            
def handle_fencing_action(key: int, arm: ArmController):
    # Map keys to functions
    action_map: Dict[int, Callable] = {
        KEY_LUNGE:      fence.lunge,
        KEY_PARRY_HIGH: fence.parry_high,
        KEY_PARRY_LOW:  fence.parry_low,
        KEY_EN_GARDE:   fence.en_garde,
        KEY_MOVE_ARM:   arm.initialise_ikpy_integration
    }
    # Execute if key exists in map
    if key in action_map:
        action_map[key]()

def main():
    combot: Combot = Combot()
    timestep = int(combot.getBasicTimeStep())
    arm: ArmController = ArmController(combot, fc.RIGHT_ARM_CONFIG)
    fence.init(arm)
    
    wb.wb_keyboard_enable(timestep)

    # ds = combot.getDevice("sword_distance_sensor")
    # ds.enable(10) # Enable with 10ms timestep

    left_wheel = combot.getDevice("wheel_left_joint")
    right_wheel = combot.getDevice("wheel_right_joint")

    # Configure motors for velocity control (set position to infinity)
    left_wheel.setPosition(float('inf'))
    right_wheel.setPosition(float('inf'))
    left_wheel.setVelocity(0.0)
    right_wheel.setVelocity(0.0)

    # Ensures sensors are enabled before reading them
    for _ in range(timestep + 1):
        combot.step(timestep)
    print("Sensors ready.")
    print(arm.create_right_arm_chain())
    # arm.get_right_joint_angles()

    done = False # Flag to ensure move_to_position is called only once

    try:
        while combot.step(timestep) != -1:

            key = wb.wb_keyboard_get_key()

            # val = ds.getValue()
            # print(f"Distance to tip target: {val}") 

            if key > 0:
                max_speed = left_wheel.getMaxVelocity()
                speed_left, speed_right = handle_movement_speed(key, max_speed)

                left_wheel.setVelocity(speed_left)
                right_wheel.setVelocity(speed_right)

                handle_fencing_action(key, arm)
            else:
                left_wheel.setVelocity(0.0)
                right_wheel.setVelocity(0.0)

            fence.check_hit()

            # combot.update_internal_position_model()
            # print(combot.get_position())

            # if not done:
            #     print("sending command to move robot to position...")
            #     combot.move_to_position(Position(3, 1, math.pi))
            #     done = True
            # move = strat.strategy7(combot)
            # if move is not None:
            #     move()

            # # enable RGBD camera
            # rgb_camera = wb.wb_robot_get_device("Astra rgb")
            # wb.wb_camera_enable(rgb_camera, timestep)
            # depth_camera = wb.wb_robot_get_device("Astra depth")
            # wb.wb_range_finder_enable(depth_camera, timestep)

            
    except KeyboardInterrupt:   
        print("Controller stopped by user.")
        pass

if __name__ == "__main__":
    main()
