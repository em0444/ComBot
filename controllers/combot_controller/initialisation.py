from typing import cast

from controller import Motor
from combot import Combot


def initialise_motors() -> None:
    robot = Combot()

    robot_part_strings = ["head_2_joint",      "head_1_joint",      "torso_lift_joint",  "arm_right_1_joint",
                      "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint", "arm_right_5_joint",
                      "arm_right_6_joint", "arm_right_7_joint", "arm_left_1_joint",  "arm_left_2_joint",
                      "arm_left_3_joint",  "arm_left_4_joint",  "arm_left_5_joint",  "arm_left_6_joint",
                      "arm_left_7_joint"]

    target_positions = [-0.3, 0.00, 0.35, 1.50, 
                       1.1, 0.00, 1.57, 1.00, 
                       0.00, 0.00, -0.80, -0.50, 
                       -0.27, 1.57,  -2.00,  1.20, 
                       2.00]


    robot_motors: list[Motor] = [cast(Motor, robot.getDevice(n)) for n in robot_part_strings]
    for part_name in robot_part_strings:
        robot_motors.append(cast(Motor, robot.getDevice(part_name)))

    for part, target_position in zip(robot_motors, target_positions):
        part.setVelocity((part.getVelocity() / 2))
        part.setPosition(target_position)
