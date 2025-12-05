# fencing_actions.py

from typing import cast
from controller import Motor, PositionSensor
# import combot
from combot import Combot
import fencing_constants as fc

arm_controller = None
combot = None

def init(controller_instance):
    global arm_controller, combot
    arm_controller = controller_instance   
    combot = controller_instance.combot
    
# Fencing Actions
def en_garde():
    """Executes the en garde fencing stance."""
    if arm_controller is None:
        print("Error: ArmController not initialized in fencing_actions!")
        return
    
    print("Executing en garde...")
    combot.body_state = (combot.body_state,"EN_GARDE")
    combot.changing_body_state = True
    arm_controller.move_to_pose(fc.EN_GARDE_ANGLES)
    combot.body_state = "EN_GARDE"
    combot.changing_body_state = False
    print("Done")

def lunge():
    """Executes the lunge fencing action."""
    if arm_controller is None:
        print("Error: ArmController not initialized in fencing_actions!")
        return
    
    print("Executing lunge...")
    combot.body_state = (combot.body_state,"LUNGE")
    combot.changing_body_state = True
    arm_controller.move_to_pose(fc.LUNGE_ANGLES)
    combot.body_state = "LUNGE"
    combot.changing_body_state = False
    print("Done")
    
def parry_high():
    """Executes the high parry fencing action."""
    if arm_controller is None:
        print("Error: ArmController not initialized in fencing_actions!")
        return
    
    print("Executing high parry...")
    combot.body_state = (combot.body_state,"PARRY_HIGH")
    combot.changing_body_state = True
    arm_controller.move_to_pose(fc.PARRY_HIGH_ANGLES)
    combot.body_state = "PARRY_HIGH"
    combot.changing_body_state = False
    print("Done")
    
def parry_low(): 
    """Executes the low parry fencing action."""
    if arm_controller is None:
        print("Error: ArmController not initialized in fencing_actions!")
        return
    
    print("Executing low parry...")
    combot.body_state = (combot.body_state,"PARRY_LOW")
    combot.changing_body_state = True
    arm_controller.move_to_pose(fc.PARRY_LOW_ANGLES)
    combot.body_state = "PARRY_LOW"
    combot.changing_body_state = False
    print("Done")

# Sword Interaction
def check_hit():
    if not arm_controller: return False
    
    # Read the sensor we added
    if "sword_tip" in arm_controller.sensors:
        val = arm_controller.sensors["sword_tip"].getValue()
        if val > 0.0: # 1.0 means collision
            print("HIT DETECTED!")
            return True
    return False

# Movement Helpers
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