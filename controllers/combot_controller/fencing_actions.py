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
ROBOT_PART_STRINGS = ["head_2_joint",      "head_1_joint",      "torso_lift_joint",  "arm_right_1_joint",
                      "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint", "arm_right_5_joint",
                      "arm_right_6_joint", "arm_right_7_joint", "arm_left_1_joint",  "arm_left_2_joint",
                      "arm_left_3_joint",  "arm_left_4_joint",  "arm_left_5_joint",  "arm_left_6_joint",
                      "arm_left_7_joint"]
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
    for part_name, position in zip(ROBOT_PART_STRINGS, positions):
        motor = cast(Motor, combot.getDevice(part_name))
        motor.setVelocity(motor.getVelocity())
        motor.setPosition(position)

# Movement Execution
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
        angles[sensor.name] = value
        
    print(angles)
    
# Fencing Actions
def en_garde():
    """Executes the en garde fencing stance."""
    get_joint_angles()
    print("Executing en garde...")
    combot.body_state = (combot.body_state,"EN_GARDE")
    combot.changing_body_state = True
    move_to_pose(EN_GARDE_POSITIONS)
    combot.body_state = "EN_GARDE"
    combot.changing_body_state = False
    print("Done")
    get_joint_angles()
     
def lunge():
    """Executes the lunge fencing action."""
    print("Executing lunge...")
    combot.body_state = (combot.body_state,"LUNGE")
    combot.changing_body_state = True
    move_to_pose(LUNGE_POSITIONS)
    combot.body_state = "LUNGE"
    combot.changing_body_state = False
    print("Done")
    get_joint_angles()
    
def parry_high():
    """Executes the high parry fencing action."""
    print("Executing high parry...")
    combot.body_state = (combot.body_state,"PARRY_HIGH")
    combot.changing_body_state = True
    move_to_pose(PARRY_HIGH_POSITIONS)
    combot.body_state = "PARRY_HIGH"
    combot.changing_body_state = False
    print("Done")
    get_joint_angles()
    
def parry_low(): 
    """Executes the low parry fencing action."""
    print("Executing low parry...")
    combot.body_state = (combot.body_state,"PARRY_LOW")
    combot.changing_body_state = True
    move_to_pose(PARRY_LOW_POSITIONS)
    combot.body_state = "PARRY_LOW"
    combot.changing_body_state = False
    print("Done")
    get_joint_angles()

# Sword Interaction
def get_sword_handle_position():
    """
    Returns the [x, y, z] coordinates of the sword's handle center.
   
    Returns: list[float]: The global coordinates [x, y, z] of the handle.
    """
    sword_node = combot.getFromDef("FENCING_SWORD_SOLID")
    
    if sword_node is None:
        print("Error: Could not find sword!")
        return [0, 0, 0]

    # 2. Get the Sword's Origin (Global Position)
    sword_origin = sword_node.getPosition()
    print("Sword Origin:", sword_origin)
    
    # 3. Get the Sword's Rotation Matrix (Global Orientation)
    # This returns a 3x3 matrix as a list of 9 floats
    sword_rotation = sword_node.getOrientation() 
    print("Sword Rotation:", sword_rotation)
    
    # 4. Calculate the Handle Offset relative to the sword origin
    # From your PROTO: translation 0 -0.25 0.06
    # This is local to the sword.
    handle_offset_local = [0, -0.25, 0.06]
    
    # 5. Apply Rotation to the Offset (Matrix Multiplication)
    # We need to rotate the offset so it matches the sword's current angle in the world.
    # Formula: Global_Offset = Rotation_Matrix * Local_Offset
    
    # Extract matrix rows (Webots returns [r0c0, r0c1, r0c2, r1c0...])
    r0 = sword_rotation[0:3] # Row 0 (X axis)
    r1 = sword_rotation[3:6] # Row 1 (Y axis)
    r2 = sword_rotation[6:9] # Row 2 (Z axis)
    
    # Dot product for rotation
    dx = r0[0]*handle_offset_local[0] + r0[1]*handle_offset_local[1] + r0[2]*handle_offset_local[2]
    dy = r1[0]*handle_offset_local[0] + r1[1]*handle_offset_local[1] + r1[2]*handle_offset_local[2]
    dz = r2[0]*handle_offset_local[0] + r2[1]*handle_offset_local[1] + r2[2]*handle_offset_local[2]
    
    # 6. Add Rotated Offset to Origin
    handle_global_pos = [
        sword_origin[0] + dx,
        sword_origin[1] + dy,
        sword_origin[2] + dz
    ]
    print("Sword Handle Global Position:", handle_global_pos)
    return handle_global_pos

FINGER_GRIPPERS = [
    # "left_hand_gripper_right_finger_joint",
    # "left_hand_gripper_left_finger_joint",
    # "right_hand_gripper_right_finger_joint",
    "right_hand_gripper_left_finger_joint"
]

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