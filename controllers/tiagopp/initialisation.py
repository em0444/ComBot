from typing import cast, List

from controller import Motor, Robot


def initialise_motors(robot: Robot) -> List[Motor]:
    robot_part_strings = ["head_2_joint", "head_1_joint", "torso_lift_joint", "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint", "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint", "wheel_left_joint", "wheel_right_joint"]

    target_positions = [0.24, -0.67, 0.09, -0.43, -0.77, 0.00, 0.96, 1.41, 1.2, 0.00, 0.74, -0.95, 0.06, 1.12, 1.45,
                        0.00, 0.00, 999, 999]

    robot_motors: list[Motor] = [cast(Motor, robot.getDevice(n)) for n in robot_part_strings]
    for part_name in robot_part_strings:
        robot_motors.append(cast(Motor, robot.getDevice(part_name)))

    for part, target_position in zip(robot_motors, target_positions):
        part.setVelocity((part.getVelocity() / 2))
        part.setPosition(target_position)

    return robot_motors