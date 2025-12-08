"""
fencing_constants.py
Stores joint names, configuration, and PRESET ANGLES.
"""

# Configuration for the right arm (The Fencing Arm)
RIGHT_ARM_CONFIG = {
    "name": "arm_right_chain",
    "base_elements": ["base_link", "arm_right_1_joint"],
    "tip_offset": [0.0, 0.75, 0.01],  # includes sword length from the wrist joint
    "joint_names": [
        "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint",
        "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint"
    ]
}

# Pose Definitions
# Standard defensive stance
EN_GARDE_ANGLES = [
    -0.3, 0.00, 0.35, 1.50, 1.1, 0.00, 1.57, 1.00, 
    0.00, 0.00, -0.80, -0.50, -0.27, 1.57, -2.00, 1.20, 2.00
]
# Extends the sword arm forward
LUNGE_ANGLES = [
    0.3, 0.0, 0.15, 1.50, -0.3, -0.52, -0.02, 0.00, 
    0.0, 0.0, -0.80, -0.2, -0.5, -0.10, 0.00, 0.20, 0.0
]
# Parry 1: Blocking High (Rotates wrist/forearm up and slightly in)
PARRY_HIGH_ANGLES = [
    0.3, 0.0, 0.25, 1.50, -0.3, -0.52, -0.02, 0.00, 
    -1.11, -1.70, -0.20, -0.50, -0.27, 1.57, -2.00, 1.20, 2.0
]
# Parry 2: Blocking Low (Rotates forearm down and slightly out)
PARRY_LOW_ANGLES = [
    0.1, 0.0, 0.25, 1.50, 0.13, -0.12, -0.12, 0.00, 
    -1.11, -1.70, -0.20, -0.50, -0.27, 1.77, -2.00, 1.20, 2.0
]

# Defines the order of parts for the pose definition lists above
FULL_BODY_PART_NAMES = [
    "head_2_joint", "head_1_joint", "torso_lift_joint",
    "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint",
    "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint",
    "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint",
    "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint"
]

#q Defines target positions and orientations for IK-based movements
IK_MOVES = {
    "LUNGE": {
        "mode":   "Z",              
        "vector": [0, 0.0, 1.0]  
    },
    "EN_GARDE": {
        "position":    [0.6, -0.1, 0.4], 
        "mode":   "Z",                   
        "vector": [1, -1, 0]        
    },
    "PARRY_HIGH": {
        "position":    [0.4,  0.10,  0.30],
        "mode":   "X",                    
        "vector": [0.0, 0.0, 1.0]         
    },
    "PARRY_LOW": {
        "position":    [0.4, 0.3, -0.4],
        "mode":   "X",
        "vector": [0, 0, -1.0]       
    }
}