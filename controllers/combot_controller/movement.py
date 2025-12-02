import math
from enum import Enum
from typing import Optional

from controller import Motor
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
    starting_position: Position = combot_obj.get_position()
    requested_delta_x: float = target_position.x - starting_position.x
    requested_delta_y: float = target_position.y - starting_position.y
    required_heading: float = (math.atan2(requested_delta_y, requested_delta_x) + (2 * math.pi)) % (2 * math.pi)
    required_distance: float = math.sqrt(requested_delta_x ** 2 + requested_delta_y ** 2)

    # Point ourselves in the right direction
    rotate_to_heading(target_heading=required_heading)

    # Start moving
    begin_moving_forward(combot, requested_delta_x, requested_delta_y)


    last_turning_direction: Optional[TurnDirection] = None
    required_turning_direction: Optional[TurnDirection] = None
    finished = False
    while not finished:

        #If we've travelled far enough, then we're done!
        combot.update_internal_position_model()
        current_position: Position = combot.get_position()
        amount_travelled = math.sqrt((current_position.x - starting_position.x) ** 2 + (current_position.y - starting_position.y) ** 2)
        if amount_travelled > required_distance:
            break

        # Wait a bit...
        for i in range(10):
            combot.step(int(combot.getBasicTimeStep()))

        # Adjust course so we're still moving on the right heading
        current_heading = combot.get_position().heading_in_radians

        turning_direction_in_radians = (required_heading - current_heading) % (2 * math.pi)
        if turning_direction_in_radians > math.pi:
            turning_direction_in_radians -= 2 * math.pi

        required_turning_direction = None
        if turning_direction_in_radians > 0.1:
            required_turning_direction = TurnDirection.LEFT
        if turning_direction_in_radians < -0.1:
            required_turning_direction = TurnDirection.RIGHT

        if not last_turning_direction == required_turning_direction: # Then we're not already adjusting our course in the correct way, so change something!
            last_turning_direction = required_turning_direction
            begin_moving_forward(combot, requested_delta_x, requested_delta_y) # Straighten back up
            if required_turning_direction is not None:
                turn(required_turning_direction, turning_speed=0.5)

    print("Combot now at destination position... Killing motors.")
    turn(TurnDirection.STOP, 0.0)

def begin_moving_forward(combot, requested_delta_x, requested_delta_y):
    print("Moving forward...")
    left_wheel_joint, right_wheel_joint = combot.getDevice("wheel_left_joint"), combot.getDevice("wheel_right_joint")
    amount_needed_to_move = math.sqrt(requested_delta_x ** 2 + requested_delta_y ** 2)
    max_wheel_speed = left_wheel_joint.getMaxVelocity()
    speed_to_move = max_wheel_speed * amount_needed_to_move / 10
    combot.getDevice("wheel_left_joint").setPosition(float('inf'))
    combot.getDevice("wheel_right_joint").setPosition(float('inf'))
    combot.getDevice("wheel_left_joint").setVelocity(speed_to_move)
    combot.getDevice("wheel_right_joint").setVelocity(speed_to_move)


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
    print("Done!")

amount_required_to_slow_down = 0.25
satisfactory_finished_distance = 0.15

def rotate_to_heading(target_heading: float):
    print(f"rotating to heading {target_heading}")
    current_heading = combot.get_position().heading_in_radians
    delta_heading = (target_heading - current_heading) % (2 * math.pi)
    if delta_heading > math.pi:
        delta_heading -= 2 * math.pi

    if abs(delta_heading < 0.05):
        print("Amount to turn too small to execute... skipping.")
        return

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
    print("Waiting for turn to be complete...")
    max_timesteps_to_wait = 200
    num_timesteps_waited = 0
    while abs(target_heading - current_heading) >= satisfactory_finished_distance * delta_heading:
        for i in range(10):
            combot.step(int(combot.getBasicTimeStep()))
            num_timesteps_waited += 1

        current_heading = combot.localisation.inertial_heading.get_heading_in_radians()
        if num_timesteps_waited >= max_timesteps_to_wait: # Maximum wait time to prevent infinite loop
            break

    print(f"Finished Turning... Current heading {current_heading}")


    timestep = combot.getBasicTimeStep()

