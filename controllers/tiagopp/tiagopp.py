"""tiagopp controller."""

from controller import Robot, Motor
from typing import cast

from controller import wb as c_webots_api #The wb package gives you all the C-like methods, but the controller package wraps most of them in nicer-to-use classes.
from controllers.tiagopp.initialisation import initialise_motors

wb = c_webots_api.wb

robot = Robot()

timestep = int(robot.getBasicTimeStep())

robot_motors = initialise_motors(robot)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
