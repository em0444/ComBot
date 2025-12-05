# IKPY Integration
import os
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import combot
from combot import Combot
import urdf_parser_py.urdf as urdf_model
import numpy as np
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
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

def get_right_joint_angles() -> dict:
    """Returns a dictionary of joint angles from all enabled sensors."""
    angles = {}
    
    for link in fc.RIGHT_SENSOR_NAMES:
        sensor = combot.getDevice(link)
        value = sensor.getValue()
        angles[sensor.name] = value
        
    print(angles)
    return angles

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

def get_opponent_position():

    """
    Returns the [x, y, z] coordinates of the opponents center.
   
    Returns: list[float]: The global coordinates [x, y, z] of the handle.
    """
    opponent_node = combot.getFromDef("OPP")
    
    if opponent_node is None:
        print("Error: Could not find opponent!")
        return [0, 0, 0]

    # 2. Get the Sword's Origin (Global Position)
    opponent_origin = opponent_node.getPosition()
    print("Opponent Origin:", opponent_origin)
    
    # 3. Get the Sword's Rotation Matrix (Global Orientation)
    # This returns a 3x3 matrix as a list of 9 floats
    opponent_rotation = opponent_node.getOrientation() 
    print("Opponent Rotation:", opponent_rotation)
    
    # 4. Calculate the Handle Offset relative to the sword origin
    # From your PROTO: translation 0 -0.25 0.06
    # This is local to the sword.
    handle_offset_local = [0, -0.25, 0.06]
    
    # 5. Apply Rotation to the Offset (Matrix Multiplication)
    # We need to rotate the offset so it matches the sword's current angle in the world.
    # Formula: Global_Offset = Rotation_Matrix * Local_Offset
    
    # Extract matrix rows (Webots returns [r0c0, r0c1, r0c2, r1c0...])
    r0 = opponent_rotation[0:3] # Row 0 (X axis)
    r1 = opponent_rotation[3:6] # Row 1 (Y axis)
    r2 = opponent_rotation[6:9] # Row 2 (Z axis)
    
    # Dot product for rotation
    dx = r0[0]*handle_offset_local[0] + r0[1]*handle_offset_local[1] + r0[2]*handle_offset_local[2]
    dy = r1[0]*handle_offset_local[0] + r1[1]*handle_offset_local[1] + r1[2]*handle_offset_local[2]
    dz = r2[0]*handle_offset_local[0] + r2[1]*handle_offset_local[1] + r2[2]*handle_offset_local[2]
    
    # 6. Add Rotated Offset to Origin
    handle_global_pos = [
        opponent_origin[0] + dx,
        opponent_origin[1] + dy,
        opponent_origin[2] + dz
    ]
    print("Opponent Global Position:", handle_global_pos)
    return handle_global_pos

def initialise_ikpy_integration(right_arm_chain: Chain):
    my_chain = right_arm_chain
    arm_angles = get_right_joint_angles()
    # I've got one disabled line at the front and four at the end
    initial_position = [0] + [angle for angle in arm_angles.values()] + [0,0,0,0]

    print("Initial Chain Position:", initial_position)

    offset_target = get_opponent_position()

    ikResults = my_chain.inverse_kinematics(
        target_position=offset_target,  
        target_orientation = [0,0,1], 
        orientation_mode="X")
    
    print("IK Results:", ikResults)

    for res in range(len(ikResults)):
        # This if check will ignore anything that isn't controllable
        if my_chain.links[res].name in fc.FULL_BODY_PART_NAMES:
            combot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])
            print("Setting {} to {}".format(my_chain.links[res].name, ikResults[res]))
    
    print("Moved arm to target position:", offset_target)
    # Visualize the IK results
    ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

    # Plot the current position of your arm
    my_chain.plot(initial_position, ax, target=offset_target)

    # And plot the target position of your arm
    my_chain.plot(ikResults, ax, target=offset_target)
    matplotlib.pyplot.show()