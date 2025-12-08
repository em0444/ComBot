"""combot controller."""
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

# Shorthand alias for the Webots API module
wb = c_webots_api.wb


def perform_test_1(combot, timestep):
    print("Performing test 1...")
    test_1_complete = False
    counter = 0

    while not test_1_complete:
        test_1_complete = combot.move_to_position(Position(2.5, 1, math.pi), counter)
        combot.step(timestep)
        counter += 1

    print("Test 1 complete!")

def main():

    combot: Combot = Combot()

    timestep = int(combot.getBasicTimeStep())
    arm: Arm = Arm(combot, fc.RIGHT_ARM_CONFIG)
    fence.init(arm)

    wb.wb_keyboard_enable(timestep)

    left_wheel = combot.getDevice("wheel_left_joint")
    right_wheel = combot.getDevice("wheel_right_joint")

    left_wheel.setPosition(float('inf'))
    right_wheel.setPosition(float('inf'))
    left_wheel.setVelocity(0.0)
    right_wheel.setVelocity(0.0)

    for _ in range(timestep + 1):
        combot.step(timestep)
    print("Sensors ready.")
    print(arm.create_right_arm_chain())

    perform_test_1(combot, timestep)


if __name__ == "__main__":
    main()
