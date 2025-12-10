import math
from enum import Enum
from typing import Optional

from controller import Motor
from combot import Combot
from shared_dataclasses import Position

global combot, target_position

def is_in_target_position():
    pass

class Movement:

    def __init__(self, combot: Combot, target_position: Position):
        # First calculate the rotation that we need to achieve to get to the positon
        self.already_performed_final_rotation = False
        self.counter_finished_at = None
        self.target_pos = target_position
        self.combot_obj = combot
        self.finished = False
        self.starting_position: Position = self.combot_obj.get_position()
        self.requested_delta_x: float = target_position.x - self.starting_position.x
        self.requested_delta_y: float = target_position.y - self.starting_position.y
        self.required_heading: float = (math.atan2(self.requested_delta_y, self.requested_delta_x) + (2 * math.pi)) % (
                    2 * math.pi)
        self.required_distance: float = math.sqrt(self.requested_delta_x ** 2 + self.requested_delta_y ** 2)
        self.last_turning_direction = None
        self.moving_backwards = False
        self.is_done = False

    def is_moving_backwards(self):
        return self.moving_backwards

    def is_done(self):
        return self.is_done

    def move_to_position(self, counter) -> bool:
        global combot, target_position
        combot, target_position = self.combot_obj, self.target_pos
        print(self.combot_obj.get_position())

        # Non-blocking code: If the counter = 0, then we've not started the turn yet
        if counter == 0:

            # If it would be less turning to move backwards, then move backwards. Otherwise, go forwards.
            heading_difference = (self.required_heading - combot.localisation.inertial_heading.get_heading_in_radians()) % (2 * math.pi)
            if abs(heading_difference) > math.pi / 2:
                print("Achieving via backwards movement")
                self.moving_backwards = True
                self.required_heading = (self.required_heading + math.pi) % 2 * math.pi
            else:
                print("Achieving via forwards movement")
                self.moving_backwards = False

            # Point ourselves in the right direction
            # rotate_to_heading(target_heading=self.required_heading)

            # Start moving
            begin_moving(combot, self.requested_delta_x, self.requested_delta_y, self.moving_backwards)


            self.required_turning_direction: Optional[TurnDirection] = None
            self.finished = False
            return False

        # Only do these checks every 30 timesteps (1 timestep = 1 counter)
        if counter % 10 == 0 and not self.finished:
            #If we've travelled far enough, then we're done!
            combot.update_internal_position_model()
            current_position: Position = combot.get_position()
            amount_travelled = math.sqrt((current_position.x - self.starting_position.x) ** 2 + (current_position.y - self.starting_position.y) ** 2)
            if amount_travelled > self.required_distance - 0.1: # Allow 0.1 metres of stopping distance for the motors to brake fully.
                print("Combot now at destination position... Killing motors.")
                turn(TurnDirection.STOP, 0.0, self.moving_backwards)
                self.finished = True
                self.counter_finished_at = counter # Find what step we finished at so we can wait a few steps to stop


            # Adjust course so we're still moving on the right heading
            current_heading = combot.localisation.inertial_heading.get_heading_in_radians()
            current_position = combot.get_position()
            delta_x: float = self.target_pos.x - current_position.x
            delta_y: float = self.target_pos.y - current_position.y
            self.required_heading: float = (math.atan2(self.requested_delta_y, self.requested_delta_x) + (2 * math.pi)) % (2 * math.pi)
            if self.moving_backwards:
                self.required_heading = (self.required_heading + math.pi) % (2 * math.pi)


            turning_direction_in_radians = (self.required_heading - current_heading) % (2 * math.pi)
            if turning_direction_in_radians > math.pi:
                turning_direction_in_radians -= 2 * math.pi

            required_turning_direction = None
            if turning_direction_in_radians > 0.1:
                required_turning_direction = TurnDirection.LEFT
                if self.moving_backwards:
                    required_turning_direction = TurnDirection.RIGHT
            if turning_direction_in_radians < -0.1:
                required_turning_direction = TurnDirection.RIGHT
                if self.moving_backwards:
                    required_turning_direction = TurnDirection.LEFT

            if not self.last_turning_direction == required_turning_direction: # Then we're not already adjusting our course in the correct way, so change something!
                self.last_turning_direction = required_turning_direction
                begin_moving(combot, self.requested_delta_x, self.requested_delta_y, self.moving_backwards) # Straighten back up
                if required_turning_direction is not None:
                    turn(required_turning_direction, turning_speed=min(abs(turning_direction_in_radians) * 5, 2), moving_backwards=self.moving_backwards) # Speed we turn proportional to how fast we're going

            # Tell them we're not done yet!
            return False

        if self.finished and counter <= self.counter_finished_at + 80: # Wait 80 timesteps after finishing so the robot has time to properly stop
            return False

        if self.finished and counter > self.counter_finished_at + 80:
            if not self.already_performed_final_rotation: # The caller is continually calling us despite already being done, don't turn again!
                self.already_performed_final_rotation = True
                rotate_to_heading(self.target_pos.heading_in_radians, self.moving_backwards) # Once all that's done, rotate to the target heading
                self.is_done = True
            return True  # And tell the caller the manouvre is complete
        return False

def begin_moving(combot, requested_delta_x, requested_delta_y, should_move_backwards):
    if should_move_backwards:
        print("Begin moving backwards.")
    else:
        print("Begin moving forwards.")
    left_wheel_joint, right_wheel_joint = combot.getDevice("wheel_left_joint"), combot.getDevice("wheel_right_joint")
    max_wheel_speed = left_wheel_joint.getMaxVelocity()
    speed_to_move = max_wheel_speed * 0.5
    if should_move_backwards: # Then move backwards
        speed_to_move = speed_to_move * -1
    combot.getDevice("wheel_left_joint").setPosition(float('inf'))
    combot.getDevice("wheel_right_joint").setPosition(float('inf'))
    combot.getDevice("wheel_left_joint").setVelocity(speed_to_move)
    combot.getDevice("wheel_right_joint").setVelocity(speed_to_move)


class TurnDirection(Enum):
    LEFT = 1
    RIGHT = 2
    STOP = 3

def turn(turning_direction: TurnDirection, turning_speed, moving_backwards) -> None:
    print(f"Initiating turn in direction {turning_direction}")
    turn_velocity = abs(combot.getDevice("wheel_left_joint").getMaxVelocity() * turning_speed * 0.1)

    wheel_left_velocity = combot.getDevice("wheel_left_joint").getVelocity()
    wheel_right_velocity = combot.getDevice("wheel_right_joint").getVelocity()

    if turning_direction == TurnDirection.LEFT: # Right wheel forwards, left wheel backwards
        if not moving_backwards:
            wheel_right_velocity += turn_velocity
            wheel_left_velocity += -turn_velocity
        else:
            wheel_right_velocity += -turn_velocity
            wheel_left_velocity += turn_velocity

    if turning_direction == TurnDirection.RIGHT: # Left wheel forwards, right wheel backwards
        if not moving_backwards:
            wheel_right_velocity += -turn_velocity
            wheel_left_velocity += turn_velocity
        else:
            wheel_right_velocity += turn_velocity
            wheel_left_velocity += -turn_velocity

    if turning_direction == TurnDirection.STOP: #Set motors to zero
        wheel_left_velocity = 0.0
        wheel_right_velocity = 0.0

    combot.getDevice("wheel_left_joint").setPosition(float('inf'))
    combot.getDevice("wheel_right_joint").setPosition(float('inf'))
    combot.getDevice("wheel_left_joint").setVelocity(wheel_left_velocity)
    combot.getDevice("wheel_right_joint").setVelocity(wheel_right_velocity)
    print("Done!")

amount_required_to_slow_down = 0.2
satisfactory_finished_rotation_distance = 0.15

def rotate_to_heading(target_heading: float, moving_backwards):
    print(f"rotating to heading {target_heading}")
    current_heading = combot.localisation.inertial_heading.get_heading_in_radians()
    delta_heading = (target_heading - current_heading) % (2 * math.pi)
    if delta_heading > math.pi:
        delta_heading -= 2 * math.pi

    if abs(delta_heading) < 0.05:
        print("Amount to turn too small to execute... skipping.")
        return

    if delta_heading == 0:
        return

    if delta_heading < 0:
        turn(TurnDirection.RIGHT, 2, moving_backwards=False) # Pass in speed - smaller turns performed slower.

    if delta_heading > 0:
        turn(TurnDirection.LEFT, 2, moving_backwards=False)

    finished_turn_procedure = False
    while not finished_turn_procedure:

        # Continue along for another 10 timesteps
        for i in range(10):
            combot.step(int(combot.getBasicTimeStep()))

        # See if we need to start slowing down
        current_heading = combot.localisation.inertial_heading.get_heading_in_radians()
        if abs(target_heading - current_heading) <= amount_required_to_slow_down:
            turn(TurnDirection.STOP, 0.0, moving_backwards)
            finished_turn_procedure = True

    #Wait for the robot to finish the turn before we do anything else
    print("Waiting for turn to be complete...")
    max_timesteps_to_wait = 75
    num_timesteps_waited = 0
    while abs(target_heading - current_heading) >= satisfactory_finished_rotation_distance * delta_heading:
        for i in range(10):
            combot.step(int(combot.getBasicTimeStep()))
            num_timesteps_waited += 1

        current_heading = combot.localisation.inertial_heading.get_heading_in_radians()
        if num_timesteps_waited >= max_timesteps_to_wait: # Maximum wait time to prevent infinite loop
            break

    print(f"Finished Turning... Current heading {current_heading}")

