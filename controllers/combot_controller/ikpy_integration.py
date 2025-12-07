# IKPY Integration
import os
from ikpy.chain import Chain
import combot
from combot import Combot
import numpy as np
import fencing_constants as fc

np.float = float  # Fix for ikpy compatibility with numpy>=1.24

# Initialise the Robot Singleton and timestep
combot = Combot()
timestep = int(combot.getBasicTimeStep())


def get_joint_limits(urdf_root):
    joint_limits = {
            joint.name: {
                "lower": joint.limit.lower,
                "upper": joint.limit.upper,
                "velocity": joint.limit.velocity
            }
            for joint in urdf_root.joint_map.values() 
            if joint.limit is not None
        }
    print("Joint Limits: \n", joint_limits)
    return joint_limits

def create_right_arm_chain(filename) -> Chain:
    # Path: Base -> Torso -> Lift -> Arm Base -> Arm Segments -> Wrist
    right_arm_chain = Chain.from_urdf_file(
        filename,
        last_link_vector=fc.RIGHT_ARM_CONFIG["tip_offset"], # end effector offset
        base_elements=fc.RIGHT_ARM_CONFIG["base_elements"],
        name=fc.RIGHT_ARM_CONFIG["name"]
    )

    print("IK Chain created with ", len(right_arm_chain.links), " links")

    return activate_ik_chain(right_arm_chain)

def activate_ik_chain(right_arm_chain: Chain):
    print("Activating IK Chain for right arm...")
    for link_id in range(len(right_arm_chain.links)):
        # This is the actual link object
        link = right_arm_chain.links[link_id]
        print("Link {}: {}".format(link_id, link.name))
        
        if link.name not in fc.FULL_BODY_PART_NAMES or  link.name =="torso_lift_joint":
            print("Disabling {}".format(link.name))
            right_arm_chain.active_links_mask[link_id] = False
        
    active_links = [right_arm_chain.links[i].name for i in range(len(right_arm_chain.links)) if right_arm_chain.active_links_mask[i]]
    print("Active links:", active_links)
    return right_arm_chain
