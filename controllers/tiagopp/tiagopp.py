"""tiagopp controller."""

from controller import Robot

from controller import wb as c_webots_api #The wb package gives you all the C-like methods, but the controller package wraps most of them in nicer-to-use classes.
wb = c_webots_api.wb

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# initialise some of the robot motors
robot_part_strings = ["head_2_joint", "head_1_joint", "torso_lift_joint", "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint", "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint", "wheel_left_joint", "wheel_right_joint"]

target_positions = [0.24, -0.67, 0.09, -0.43, -0.77, 0.00, 0.96, 1.41, 1.2, 0.00, 0.74, -0.95, 0.06, 1.12,  1.45,  0.00, 0.00, 999, 999]

robot_parts = []
for part_name in robot_part_strings:
    robot_parts.append(robot.getDevice(part_name))

for part, target_position in zip(robot_parts, target_positions):
    part.setVelocity((part.getVelocity() / 2)) #TODO add proper type checking so this isn't scary
    part.setPosition(target_position)

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
