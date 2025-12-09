"""combot controller."""
import csv
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
    """
    """
    print("Performing test 1...")
    test_1_complete = False
    counter = 0
    results = []
    robot_supervisor = combot.getFromDef("FENCER")

    while not test_1_complete:
        test_1_complete = combot.move_to_position(Position(2.5, 1, 3 * math.pi / 2), counter)
        combot.step(timestep)
        counter += 1

        time = combot.getTime()
        real_position = robot_supervisor.getField("translation")
        estimated_position_from_localisation = combot.get_position()

        results.append({'time': time, 'real_position_x': real_position.value[0], 'real_position_y': real_position.value[1], 'estimated_position_from_localisation_x': estimated_position_from_localisation.x, 'estimated_position_from_localisation_y': estimated_position_from_localisation.y, 'has_finished': test_1_complete})

    with open("test_1_log.csv", "w") as f:
        writer = csv.DictWriter(f, fieldnames=["time", "real_position_x", "real_position_y", "estimated_position_from_localisation_x", "estimated_position_from_localisation_y", "has_finished"])
        writer.writeheader()
        writer.writerows(results)

    print("Test 1 complete!")

def perform_test_2(combot, timestep):
    """
    """
    print("Performing test 2...")
    test_2_complete = False
    counter = 0
    results = []
    robot_supervisor = combot.getFromDef("FENCER")

    while not test_2_complete:
        test_2_complete = combot.move_to_position(Position(-2.5, -0.5, 0), counter)
        combot.step(timestep)
        counter += 1

        time = combot.getTime()
        real_position = robot_supervisor.getField("translation")
        estimated_position_from_localisation = combot.get_position()

        results.append({'time': time, 'real_position_x': real_position.value[0], 'real_position_y': real_position.value[1], 'estimated_position_from_localisation_x': estimated_position_from_localisation.x, 'estimated_position_from_localisation_y': estimated_position_from_localisation.y, 'has_finished': test_2_complete})

    with open("test_2_log.csv", "w") as f:
        writer = csv.DictWriter(f, fieldnames=["time", "real_position_x", "real_position_y", "estimated_position_from_localisation_x", "estimated_position_from_localisation_y", "has_finished"])
        writer.writeheader()
        writer.writerows(results)

    print("Test 2 complete!")


def perform_test_3(combot, timestep):
    print("Performing test 3...")
    results= []
    robot_supervisor = combot.getFromDef("FENCER")

    print("Performing manouvre 1...")
    manouvre_complete = False
    counter = 0
    while not manouvre_complete:
        manouvre_complete = combot.move_to_position(Position(1, 0.1, 0), counter)
        combot.step(timestep)
        counter += 1

        time = combot.getTime()
        real_position = robot_supervisor.getField("translation")
        estimated_position_from_localisation = combot.get_position()

        results.append({'time': time, 'real_position_x': real_position.value[0], 'real_position_y': real_position.value[1], 'estimated_position_from_localisation_x': estimated_position_from_localisation.x, 'estimated_position_from_localisation_y': estimated_position_from_localisation.y, 'manouvres_completed': 0})

    print("Manouvre 1 complete!")


    print("Performing manouvre 2...")
    manouvre_complete = False
    counter = 0
    while not manouvre_complete:
        manouvre_complete = combot.move_to_position(Position(-1, -1, 0), counter)
        combot.step(timestep)
        counter += 1

        time = combot.getTime()
        real_position = robot_supervisor.getField("translation")
        estimated_position_from_localisation = combot.get_position()

        results.append({'time': time, 'real_position_x': real_position.value[0], 'real_position_y': real_position.value[1], 'estimated_position_from_localisation_x': estimated_position_from_localisation.x, 'estimated_position_from_localisation_y': estimated_position_from_localisation.y, 'manouvres_completed': 1})

    print("Manouvre 2 complete!")

    print("Performing manouvre 3...")
    manouvre_complete = False
    counter = 0
    while not manouvre_complete:
        manouvre_complete = combot.move_to_position(Position(0, 0, 0.1), counter)
        combot.step(timestep)
        counter += 1

        time = combot.getTime()
        real_position = robot_supervisor.getField("translation")
        estimated_position_from_localisation = combot.get_position()

        results.append({'time': time, 'real_position_x': real_position.value[0], 'real_position_y': real_position.value[1],
                        'estimated_position_from_localisation_x': estimated_position_from_localisation.x,
                        'estimated_position_from_localisation_y': estimated_position_from_localisation.y,
                        'manouvres_completed': 2})

    print("Manouvre 3 complete!")

    with open("test_3_log.csv", "w") as f:
        writer = csv.DictWriter(f, fieldnames=["time", "real_position_x", "real_position_y", "estimated_position_from_localisation_x", "estimated_position_from_localisation_y", "manouvres_completed"])
        writer.writeheader()
        writer.writerows(results)


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

    # Assume en garde and wait a bit
    fence.en_garde()
    for i in range(100):
        combot.step(timestep)

    # Perform the localisation tests
    # perform_test_1(combot, timestep)
    # perform_test_2(combot, timestep)
    perform_test_3(combot, timestep)


if __name__ == "__main__":
    main()
