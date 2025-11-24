# fencing_actions.py

from typing import cast
from controller import Motor, PositionSensor
import combot
from combot import Combot

# Initialise the Robot Singleton and timestep
combot = Combot()
timestep = int(combot.getBasicTimeStep())

# List of all joint sensor names to be enabled
SENSOR_NAMES = [
    "head_2_joint_sensor",       "head_1_joint_sensor",       "torso_lift_joint_sensor",
    "arm_right_1_joint_sensor",  "arm_right_2_joint_sensor",  "arm_right_3_joint_sensor",
    "arm_right_4_joint_sensor",  "arm_right_5_joint_sensor",  "arm_right_6_joint_sensor",
    "arm_right_7_joint_sensor",  "arm_left_1_joint_sensor",   "arm_left_2_joint_sensor",
    "arm_left_3_joint_sensor",   "arm_left_4_joint_sensor",   "arm_left_5_joint_sensor",
    "arm_left_6_joint_sensor",   "arm_left_7_joint_sensor"
]

# Pose Definitions
# Standard defensive stance
EN_GARDE_POSITIONS = [-0.3, 0.00, 0.35, 1.50, 
                       1.1, 0.00, 1.57, 1.00, 
                       0.00, 0.00, -0.80, -0.50, 
                       -0.27, 1.57,  -2.00,  1.20, 
                       2.00]

# Extends the sword arm forward
LUNGE_POSITIONS = [0.3, 0.0, 0.15, 1.50, 
                    -0.3, -0.52, -0.02, 0.00, 
                    0.0, 0.0, -0.80, -0.2, 
                    -0.5, -0.10, 0.00, 0.20, 
                    0.0]

# Parry 1: Blocking High (Rotates wrist/forearm up and slightly in)
PARRY_HIGH_POSITIONS = [0.3, 0.0, 0.25, 1.50, 
                        -0.3, -0.52, -0.02, 0.00, 
                        -1.11, -1.70, -0.20, -0.50, 
                       -0.27, 1.57,  -2.00,  1.20, 
                       2.0]

# Parry 2: Blocking Low (Rotates forearm down and slightly out)
PARRY_LOW_POSITIONS = [0.1, 0.0, 0.25, 1.50, 
                        0.13, -0.12, -0.12, 0.00, 
                        -1.11, -1.70, -0.20, -0.50, 
                       -0.27, 1.77,  -2.00,  1.20, 
                       2.0]

# Action Execution
def move_to_pose(positions: list[float]):
    """Moves the robot to the specified joint positions.

    Args:
        positions (list[float]): List of target joint positions.
    """
    robot_part_strings = ["head_2_joint",      "head_1_joint",      "torso_lift_joint",  "arm_right_1_joint",
                      "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint", "arm_right_5_joint",
                      "arm_right_6_joint", "arm_right_7_joint", "arm_left_1_joint",  "arm_left_2_joint",
                      "arm_left_3_joint",  "arm_left_4_joint",  "arm_left_5_joint",  "arm_left_6_joint",
                      "arm_left_7_joint"]

    for part_name, position in zip(robot_part_strings, positions):
        motor = cast(Motor, combot.getDevice(part_name))
        motor.setVelocity(motor.getVelocity())
        motor.setPosition(position)

def enable_sensors():
    """Enables all joint position sensors."""
    print("Enabling all joint sensors...")
    for sensor in SENSOR_NAMES:
        try:
            sensor = combot.getDevice(sensor)
            sensor.enable(timestep)
        except AttributeError:
            print(f"Warning: Sensor '{sensor}' not found on robot.")

def get_joint_angles():
    """Returns a dictionary of joint angles from all enabled sensors."""
    angles = {}
    for sensor in SENSOR_NAMES:
        sensor = combot.getDevice(sensor)
        value = sensor.getValue()
        angles[sensor] = value
        
    print(angles)
    

# Fencing Actions
def en_garde():
    """Executes the en garde fencing stance."""
    get_joint_angles()
    print("Executing en garde...")
    combot.current_state = (combot.current_state,"EN_GARDE")
    combot.changing_state = True
    move_to_pose(EN_GARDE_POSITIONS)
    combot.current_state = "EN_GARDE"
    combot.changing_state = False
    print("Done")
    get_joint_angles()
     
def lunge():
    """Executes the lunge fencing action."""
    print("Executing lunge...")
    combot.current_state = (combot.current_state,"LUNGE")
    combot.changing_state = True
    move_to_pose(LUNGE_POSITIONS)
    combot.current_state = "LUNGE"
    combot.changing_state = False
    print("Done")
    get_joint_angles()
    
def parry_high():
    """Executes the high parry fencing action."""
    print("Executing high parry...")
    combot.current_state = (combot.current_state,"PARRY_HIGH")
    combot.changing_state = True
    move_to_pose(PARRY_HIGH_POSITIONS)
    combot.current_state = "PARRY_HIGH"
    combot.changing_state = False
    print("Done")
    get_joint_angles()
    
def parry_low(): 
    """Executes the low parry fencing action."""
    print("Executing low parry...")
    combot.current_state = (combot.current_state,"PARRY_LOW")
    combot.changing_state = True
    move_to_pose(PARRY_LOW_POSITIONS)
    combot.current_state = "PARRY_LOW"
    combot.changing_state = False
    print("Done")
    get_joint_angles()
