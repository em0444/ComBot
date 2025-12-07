"""
ikpy_integration.py
Handles IKPY (Inverse Kinematics Python) chain creation and configuration.
Converts URDF robot model to IK chain for computing joint angles to reach targets.
"""

import os
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np

# import combot
from combot import Combot
import fencing_constants as fc

np.float = float  # Fix for ikpy compatibility with numpy>=1.24

# Initialise the Robot Singleton and timestep
combot = Combot()
timestep = int(combot.getBasicTimeStep())

# IK Chain Creation
def create_right_arm_chain(urdf_filename) -> Chain:
    """Create an inverse kinematics chain from the robot's URDF file."""
    right_arm_chain = Chain.from_urdf_file(
        urdf_filename,
        last_link_vector=fc.RIGHT_ARM_CONFIG["tip_offset"], # end effector (sword) offset relative to the last joint (wrist)
        base_elements=fc.RIGHT_ARM_CONFIG["base_elements"],
        name=fc.RIGHT_ARM_CONFIG["name"]
    )

    print("IK Chain created with", len(right_arm_chain.links), "links")

    # Configure which joints are controllable and active for IK
    return activate_ik_chain(right_arm_chain)

def activate_ik_chain(right_arm_chain: Chain):
    """Configure which links in the IK chain are active for inverse kinematics."""
    print("onfiguring IK chain - marking controllable joints...")
    
    # Iterate through all links in the chain
    for link_id in range(len(right_arm_chain.links)):
        # Get the link object at this position in the chain
        link = right_arm_chain.links[link_id]
        print(f"  Link {link_id}: {link.name}")
        
        # Check if this link should be active for IK
        if link.name not in fc.FULL_BODY_PART_NAMES or  link.name =="torso_lift_joint":
            print(f"    -> Disabling {link.name} (not controllable)")
            right_arm_chain.active_links_mask[link_id] = False
        
    # Log which links are active for debugging/verification
    active_links = [
        right_arm_chain.links[i].name 
        for i in range(len(right_arm_chain.links)) 
        if right_arm_chain.active_links_mask[i]
    ]
    print(f"Active links for IK: {active_links}")
    print(f"Total active joints: {sum(right_arm_chain.active_links_mask)}")
    
    return right_arm_chain
