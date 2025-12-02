import math
from enum import Enum

from controllers.combot_controller.combot import Combot
from controllers.combot_controller.shared_dataclasses import Position

global combot, target_position

def is_in_target_position():
    pass


def move_to_position(combot_obj: Combot, target_pos: Position) -> None:
    global combot, target_position
    combot, target_position = combot_obj, target_pos
    print(combot_obj.get_position())


    # First calculate the rotation that we need to achieve to get to the positon
    current_position: Position = combot_obj.get_position()
    requested_delta_x: float = target_position.x - current_position.x
    requested_delta_y: float = target_position.y - current_position.y
    required_heading: float = (math.atan2(requested_delta_y, requested_delta_x) + (2 * math.pi)) % (2 * math.pi)

    rotate_to_heading(target_heading=required_heading)

    # while not can_finish:
    #
    #     left_wheel_joint, right_wheel_joint = combot.getDevice("wheel_left_joint"), combot.getDevice("wheel_right_joint")
    #     max_velocity = combot.getDevice("wheel_left_joint").getMaxVelocity()
    #
    #
    #
    #     if is_in_target_position():
    #         previous_times_combot_in_correct_position += 1
    #     if previous_times_combot_in_correct_position == 3:
    #         can_finish = True

class TurnDirection(Enum):
    LEFT = 1
    RIGHT = 2
    STOP = 3

def turn(turning_direction: TurnDirection, turning_speed) -> None:
    print(f"Initiating turn in direction {turning_direction}")
    turn_velocity = combot.getDevice("wheel_left_joint").getMaxVelocity() * turning_speed * 0.1

    wheel_left_velocity = combot.getDevice("wheel_left_joint").getVelocity()
    wheel_right_velocity = combot.getDevice("wheel_right_joint").getVelocity()

    if turning_direction == TurnDirection.LEFT: # Right wheel forwards, left wheel backwards
        wheel_right_velocity += turn_velocity
        wheel_left_velocity += -turn_velocity

    if turning_direction == TurnDirection.RIGHT: # Left wheel forwards, right wheel backwards
        wheel_right_velocity += -turn_velocity
        wheel_left_velocity += turn_velocity

    if turning_direction == TurnDirection.STOP: #Set motors to zero
        wheel_left_velocity = 0.0
        wheel_right_velocity = 0.0

    combot.getDevice("wheel_left_joint").setPosition(float('inf'))
    combot.getDevice("wheel_right_joint").setPosition(float('inf'))
    combot.getDevice("wheel_left_joint").setVelocity(wheel_left_velocity)
    combot.getDevice("wheel_right_joint").setVelocity(wheel_right_velocity)

amount_required_to_slow_down = 0.25
satisfactory_finished_distance = 0.1

def rotate_to_heading(target_heading: float):
    print(f"rotating to heading {target_heading}")
    current_heading = combot.get_position().heading_in_radians
    delta_heading = target_heading - current_heading

    if delta_heading == 0:
        return

    if delta_heading < 0:
        turn(TurnDirection.RIGHT, abs(delta_heading / 2 * math.pi)) # Pass in speed - smaller turns performed slower.

    if delta_heading > 0:
        turn(TurnDirection.LEFT, abs(delta_heading / 2 * math.pi))

    finished_turn_procedure = False
    while not finished_turn_procedure:

        # Continue along for another 10 timesteps
        for i in range(10):
            combot.step(int(combot.getBasicTimeStep()))

        # See if we need to start slowing down
        combot.update_internal_position_model()
        current_heading = combot.localisation.inertial_heading.get_heading_in_radians()
        if abs(target_heading - current_heading) <= amount_required_to_slow_down * delta_heading:
            turn(TurnDirection.STOP, 0.0)
            finished_turn_procedure = True

    #Wait for the robot to finish the turn before we do anything else
    while abs(target_heading - current_heading) >= satisfactory_finished_distance * delta_heading:
        for i in range(10):
            combot.step(int(combot.getBasicTimeStep()))

        current_heading = combot.localisation.inertial_heading.get_heading_in_radians()

    print(f"Finished Turning... Current heading {current_heading}")


    timestep = combot.getBasicTimeStep()

