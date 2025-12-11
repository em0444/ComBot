"""
fencing_actions.py
Implements ComBot fencing actions (lunge, parry, en garde) and movement controls.
Provides high-level fencing behavior that delegates to arm.py for IK calculations.
"""

from typing import cast
from controller import Motor, PositionSensor

from combot import Combot
import fencing_constants as fc

# Global state of arm controller and robot body
arm = None
combot = None

def init(controller_instance):
    """Initialise the fencing_actions module with arm and combot references."""
    global arm, combot
    arm = controller_instance   
    combot = controller_instance.combot
    print("Fencing actions module initialised.")

# Helper functions
def execute_fencing_action(action_name: str, angle_preset: list, ik_config: dict) -> None:
    """Generic fencing action executor
    
    Args:
        action_name: Name of action for logging (e.g., "EN_GARDE", "LUNGE")
        angle_preset: Preset angles from fencing_constants (e.g., fc.EN_GARDE_ANGLES)
        ik_config: IK configuration dict with "position", "mode", "vector" keys
    """
    if arm is None:
        print("Error: Arm not initialized in fencing_actions!")
        return
    
    print(f"Executing {action_name}...")
    
    # Track state transition
    combot.body_state = (combot.body_state, action_name)
    combot.changing_body_state = True
    
    # Move to preset pose using direct angle control
    arm.move_to_pose(angle_preset)
    
    # Fine-tune position using IK
    # Special case: LUNGE tracks opponent in real-time
    if action_name == "LUNGE":
        target_position = arm.get_opponent_target()
    else:
        target_position = ik_config["position"]
    
    arm.move_to_target(
    target_position=target_position,
    orientation_mode=ik_config["mode"],
    target_orientation=ik_config["vector"]
    )
    
    # Update final state
    combot.body_state = action_name
    combot.changing_body_state = False
    #print(f"{action_name} complete.")

def set_wheel_velocity(velocity: float) -> None:
    """Set both wheels to the same velocity."""
    left_wheel = combot.getDevice("wheel_left_joint")
    right_wheel = combot.getDevice("wheel_right_joint")
    
    # Enable continuous rotation mode (position = infinity)
    left_wheel.setPosition(float('inf'))
    right_wheel.setPosition(float('inf'))
    
    # Apply velocity to both wheels
    left_wheel.setVelocity(velocity)
    right_wheel.setVelocity(velocity)

def set_base_state(state_name: str) -> None:
    """Update the robot's base locomotion state."""
    combot.base_state = (combot.base_state, state_name)
    combot.changing_base_state = True
    combot.base_state = state_name
    combot.changing_base_state = False

# Fencing Actions
def en_garde():
    """Execute the en garde (ready) fencing stance."""
    execute_fencing_action("EN_GARDE", fc.EN_GARDE_ANGLES, fc.IK_MOVES["EN_GARDE"])

def lunge():
    """Execute the lunge fencing action - aggressive forward strike."""
    execute_fencing_action("LUNGE", fc.LUNGE_ANGLES, fc.IK_MOVES["LUNGE"])

def parry_high():
    """Execute the high parry fencing action - defensive block."""
    execute_fencing_action("PARRY_HIGH", fc.PARRY_HIGH_ANGLES, fc.IK_MOVES["PARRY_HIGH"])

def parry_low(): 
    """Execute the low parry fencing action - defensive block."""
    execute_fencing_action("PARRY_LOW", fc.PARRY_LOW_ANGLES, fc.IK_MOVES["PARRY_LOW"])

# Sword Interaction
def check_hit() -> bool:
    "Check if the sword has made contact with the opponent."
    if not arm: 
        return False
    
    # Read the sensor we added
    if "sword_tip" in arm.sensors:
        sensor_value = arm.sensors["sword_tip"].getValue()
        # Sensor returns > 0.0 when in contact (1.0 = collision)
        if sensor_value > 0.0:
            print("HIT DETECTED!")
            return True
    return False

# Movement Helpers
def move_forward():
    """Move the robot base forward at maximum velocity."""
    #max_velocity = combot.getDevice("wheel_left_joint").getMaxVelocity()
    #set_wheel_velocity(max_velocity)
    set_base_state("FORWARD")

def move_backward():
    """Move the robot base backward at maximum velocity."""
    #max_velocity = combot.getDevice("wheel_left_joint").getMaxVelocity()
    #set_wheel_velocity(-max_velocity)
    set_base_state("BACKWARD")

def move_stop():
    """Stop the robot base completely."""
    #set_wheel_velocity(0)
    set_base_state("STILL")
