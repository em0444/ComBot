# fencing_actions.py

from typing import cast
from controller import Motor
import combot
from combot import Combot

combot = Combot() # Treating robot as a singleton

# Assuming the right arm is the dominant sword arm

# Pose Definitions
# Use the "en garde" values from the previous turn for the base
EN_GARDE_POSITIONS = [-0.3, 0.00, 0.35, 1.50, 
                       1.1, 0.00, 1.57, 1.00, 
                       0.00, 0.00, -0.80, -0.50, 
                       -0.27, 1.57,  -2.00,  1.20, 
                       2.00]

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
    # Sets joint positions
    robot_part_strings = ["head_2_joint",      "head_1_joint",      "torso_lift_joint",  "arm_right_1_joint",
                      "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint", "arm_right_5_joint",
                      "arm_right_6_joint", "arm_right_7_joint", "arm_left_1_joint",  "arm_left_2_joint",
                      "arm_left_3_joint",  "arm_left_4_joint",  "arm_left_5_joint",  "arm_left_6_joint",
                      "arm_left_7_joint"]

    for part_name, position in zip(robot_part_strings, positions):
        motor = cast(Motor, combot.getDevice(part_name))
        motor.setVelocity(motor.getVelocity())
        motor.setPosition(position)

# Movement Execution

# Functions
def en_garde():
    print("Executing en garde...")
    combot.body_state = (combot.body_state,"EN_GARDE")
    combot.changing_body_state = True
    move_to_pose(EN_GARDE_POSITIONS)
    combot.body_state = "EN_GARDE"
    combot.changing_body_state = False
    print("Done")
     
def lunge():
    print("Executing lunge...")
    combot.body_state = (combot.body_state,"LUNGE")
    combot.changing_body_state = True
    move_to_pose(LUNGE_POSITIONS)
    combot.body_state = "LUNGE"
    combot.changing_body_state = False
    print("Done")
    
def parry_high():
    print("Executing high parry...")
    combot.body_state = (combot.body_state,"PARRY_HIGH")
    combot.changing_body_state = True
    move_to_pose(PARRY_HIGH_POSITIONS)
    combot.body_state = "PARRY_HIGH"
    combot.changing_body_state = False
    print("Done")
    
def parry_low():  
    print("Executing low parry...")
    combot.body_state = (combot.body_state,"PARRY_LOW")
    combot.changing_body_state = True
    move_to_pose(PARRY_LOW_POSITIONS)
    combot.body_state = "PARRY_LOW"
    combot.changing_body_state = False
    print("Done")

def move_forward():
    combot.base_state = (combot.base_state,"FORWARD")
    combot.changing_base_state = True
    combot.getDevice("wheel_left_joint").setPosition(float('inf'))
    combot.getDevice("wheel_right_joint").setPosition(float('inf'))
    combot.getDevice("wheel_left_joint").setVelocity(combot.getDevice("wheel_left_joint").getMaxVelocity())
    combot.getDevice("wheel_right_joint").setVelocity(combot.getDevice("wheel_right_joint").getMaxVelocity())
    combot.base_state = "FORWARD"
    combot.changing_base_state = False

def move_backward():
    combot.base_state = (combot.base_state,"BACKWARDS")
    combot.changing_base_state = True
    combot.getDevice("wheel_left_joint").setPosition(float('inf'))
    combot.getDevice("wheel_right_joint").setPosition(float('inf'))
    combot.getDevice("wheel_left_joint").setVelocity(-combot.getDevice("wheel_left_joint").getMaxVelocity())
    combot.getDevice("wheel_right_joint").setVelocity(-combot.getDevice("wheel_right_joint").getMaxVelocity())
    combot.base_state = "BACKWARD"
    combot.changing_base_state = False

def move_stop():
    combot.base_state = (combot.base_state,"STILL")
    combot.changing_base_state = True
    combot.getDevice("wheel_left_joint").setPosition(float('inf'))
    combot.getDevice("wheel_right_joint").setPosition(float('inf'))
    combot.getDevice("wheel_left_joint").setVelocity(0)
    combot.getDevice("wheel_right_joint").setVelocity(0)
    combot.base_state = "STILL"
    combot.changing_base_state = False