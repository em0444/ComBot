# Arm controller wrapper — uses existing fencing_actions and ikpy_integration modules
import os
from typing import List, Dict, TYPE_CHECKING

import matplotlib
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from combot import Combot
import fencing_constants as fc
import ikpy_integration as ik

class Arm:
    """
    High-level API to control ComBot arms using Inverse Kinematics, Forward Kinematics, and sensor data.
    Handles arm initialisation, movement, targeting, and position tracking.
    """

    def __init__(self, combot: Combot, config: dict):
        self.combot = combot
        self.timestep = int(combot.getBasicTimeStep())
        self.config = config
        self.motors: Dict = {}
        self.sensors: Dict = {}

        self._init_all_sensors()
        self.urdf_file = self._check_urdf()
        self.right_arm_chain = self.create_right_arm_chain()

    def _init_all_sensors(self) -> None:
        """Initialises all joint motors and position sensors including the sword touch sensor"""
        print("Enabling all joint motors and position sensors...")
        for motor_name in fc.FULL_BODY_PART_NAMES:
            try:
                motor = self.combot.getDevice(motor_name)
                self.motors[motor_name] = motor # Saving motor reference for future use
                sensor = motor.getPositionSensor()
                if sensor: 
                    sensor.enable(self.timestep)
                    self.sensors[motor_name + "_sensor"] = sensor
            except AttributeError:
                pass
        
        # Ensures sensors are enabled before reading them
        for _ in range(self.timestep + 1):
            self.combot.step(self.timestep)

        # Enable sword touch sensor for hit detection
        print("Enabling sword touch sensor...")
        sword_sensor = self.combot.getDevice("sword_tip_sensor")
        sword_sensor.enable(self.timestep)
        self.sensors["sword_tip"] = sword_sensor
        print("All sensors ready.")

    def _check_urdf(self) -> str:
        """Check if URDF file exists; generate if missing."""
        filename = "player_urdf.urdf"
        if not os.path.exists(filename):
            print(f"URDF file not found. Generating {filename}...")
            with open(filename, "w") as file:
                file.write(self.combot.getUrdf())
        else:
            print(f"Found existing {filename}, skipping generation.")
    
        return filename

    # Sensor state helper
    def get_right_joint_angles(self) -> dict:
        """ Get all right arm joint angles from enabled sensors."""
        angles = {}

        # Iterate through sensors dictionary and filter for right arm sensors
        for sensor_key, sensor in self.sensors.items():
            if "right" in sensor_key.lower():
                value = sensor.getValue()
                angles[sensor_key] = value

        return angles

    # Kinematics safety checks
    def is_solution_safe(self, joint_configuration) -> bool:
        """
        Checks if the intelligent solution places any part of the sword (from Wrist to Tip) inside the robot body
        """
        # Get the positions of all links for this solution
        frames = self.right_arm_chain.forward_kinematics(joint_configuration, full_kinematics=True)
        
        # Extract Wrist and Tip positions
        wrist_position = frames[6][:3, 3] # Link 7: arm_right_7_joint
        tip_position   = frames[-1][:3, 3] # Last joint: sword tip offset

        SAFE_RADIUS = 0.28  # 28cm radius (Body is ~25cm, adding buffer)
        
        # Check 5 points along the blade (0%, 25%, 50%, 75%, 100%)
        for i in range(5):
            alpha = i / 4.0  # 0.0, 0.25, 0.5, 0.75, 1.0
            
            # Linear Interpolation between Wrist and Tip
            check_x = wrist_position[0] * (1 - alpha) + tip_position[0] * alpha
            check_y = wrist_position[1] * (1 - alpha) + tip_position[1] * alpha
            
            # Cylinder Check (Ignore Z height, just check horizontal radius)
            dist_from_center = (check_x**2 + check_y**2)**0.5

            # If any point is inside the body cylinder, the move is unsafe.
            if dist_from_center < SAFE_RADIUS:
                print(f"Blade segment {alpha*100}% passes through body!")
                # print("Time elapsed: ", self.combot.get_elapsed_time()) 
                return False # UNSAFE

        return True # SAFE
    
    # Direct arm control (delegates to fencing_actions)
    def move_to_pose(self, positions: List[float]) -> None:
        """Move arm joints to specified positions."""
        for part_name, position in zip(fc.FULL_BODY_PART_NAMES, positions):
            motor = self.motors[part_name]
            motor.setVelocity(motor.getMaxVelocity())
            motor.setPosition(position)

    # Targeting for sword
    def get_opponent_target(self) -> List[float]:
        """Calculate the target position on opponent's body for sword strikes."""
        
        # Get opponent node reference
        opponent_node = self.combot.getFromDef("FENCER")

        if opponent_node is None:
            # Try finding it one last time (in case it spawned late)
            opponent_node = self.combot.getFromDef("FENCER")
            if opponent_node is None:
                print("Error: Could not find opponent!")
                return [0, 0, 0]
        
        opponent_position = opponent_node.getPosition()
        opponent_rotation = opponent_node.getOrientation()
        
        # Calibrated offset that accurately targets opponent's torso for sword strikes
        opponent_offset_local = [0.15, -0.1, 0.35]
        
        # Extract rotation matrix rows (Webots returns flattened 3x3 matrix)
        r0 = opponent_rotation[0:3] # Row 0 (X axis)
        r1 = opponent_rotation[3:6] # Row 1 (Y axis)
        r2 = opponent_rotation[6:9] # Row 2 (Z axis)
        
        # Apply rotation to local offset: Global_Offset = Rotation_Matrix * Local_Offset
        dx = r0[0]*opponent_offset_local[0] + r0[1]*opponent_offset_local[1] + r0[2]*opponent_offset_local[2]
        dy = r1[0]*opponent_offset_local[0] + r1[1]*opponent_offset_local[1] + r1[2]*opponent_offset_local[2]
        dz = r2[0]*opponent_offset_local[0] + r2[1]*opponent_offset_local[1] + r2[2]*opponent_offset_local[2]
        
        # Add Rotated Offset to initial position
        opponent_target_position = [
            opponent_position[0] + dx,
            opponent_position[1] + dy,
            opponent_position[2] + dz
        ]

        return opponent_target_position
    
    def get_sword_target_position(self, fence_position: List[float]) -> List[float]:
        """Calculate the target position relative to current robot for fencing actions excluding lunge."""
        
        robot_node = self.combot.getSelf()
        robot_position = robot_node.getPosition()
        robot_rotation = robot_node.getOrientation() # 3x3 Matrix
        
        # Extract rotation matrix rows
        r0 = robot_rotation[0:3] # Row 0 (X axis)
        r1 = robot_rotation[3:6] # Row 1 (Y axis)
        r2 = robot_rotation[6:9] # Row 2 (Z axis)
        
        # Apply rotation to local offset: Global_Offset = Rotation_Matrix * Local_Offset
        dx = r0[0]*fence_position[0] + r0[1]*fence_position[1] + r0[2]*fence_position[2]
        dy = r1[0]*fence_position[0] + r1[1]*fence_position[1] + r1[2]*fence_position[2]
        dz = r2[0]*fence_position[0] + r2[1]*fence_position[1] + r2[2]*fence_position[2]
        
        # Add Rotated Offset to initial position
        sword_target_position = [
            robot_position[0] + dx,
            robot_position[1] + dy,
            robot_position[2] + dz
        ]
        
        return sword_target_position

    # Inverse Kinematics (IK) Movement
    def move_to_target(self, target_position: List[float],
                        orientation_mode: str = "Z", 
                        target_orientation: List[float] = [0,0,1]) -> None:
        """Move arm to target position using inverse kinematics."""
        arm_angles = self.get_right_joint_angles()

        # Build angle list matching chain structure
        current_angles = [0] + [angle for angle in arm_angles.values()] + [0]*4
        current_angles = current_angles[:len(self.right_arm_chain.links)]
        print("Initial Chain Position:", current_angles)
        
        robot_node = self.combot.getSelf()
        robot_position = np.array(robot_node.getPosition())
        robot_rotation = np.array(robot_node.getOrientation()).reshape(3, 3)

        # For Player coordinate transformation;    
        # Converts global target to local target relative to the robot
        target_position_global = np.array(target_position)
        target_position_local = np.dot(robot_rotation.T, (target_position_global - robot_position))

        # Solve IK problem to reach target 
        ik_results = self.right_arm_chain.inverse_kinematics(
            target_position=target_position_local,  
            target_orientation = target_orientation,
            orientation_mode=orientation_mode,
            initial_position=current_angles
            )
        
         # Stops collisions with the robot body
        if not self.is_solution_safe(ik_results):
            print("Kinematics cancelled...")
            return
        
        print("IK Chain Results:", ik_results)

        # Apply IK solution to controllable joints
        for i, angle in enumerate(ik_results):
            link_name = self.right_arm_chain.links[i].name
            
            # Only set position for controllable joints
            if link_name in self.motors:
                self.combot.getDevice(link_name).setPosition(angle)

        print("Moved arm to target position:", target_position)

        # Visualise IK solution
        # self.visualise_ik(current_angles, ik_results, target_position)

    def visualise_ik(self, current_angles: List[float], 
                      ik_results: List[float], 
                      target_position: List[float]) -> None:
        """Visualise IK solution using matplotlib; Shows arm before and after IK, with target point marked."""
        ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

        # Plot initial position (blue)
        self.right_arm_chain.plot(current_angles, ax, target=target_position)

        # Plot final position (orange)
        self.right_arm_chain.plot(ik_results, ax, target=target_position)
        
        matplotlib.pyplot.show()

    # Forward kinematics (FK) position calculation
    def get_sword_tip(self):
        """
        Calculates the global sword tip position using the Supervisor API.
        """
        # Get wrist node (defined in robot PROTO)
        wrist_node = self.combot.getFromDef("PLAYER_WRIST")
        
        if wrist_node is None:
            print("Error: PLAYER_WRIST node not found!")
            return [0, 0, 0]

        wrist_position = wrist_node.getPosition()
        wrist_rotation = wrist_node.getOrientation().reshape(3, 3) 

        # Define the sword offset (Vector from Wrist -> Tip)
        sword_length = 0.9 
        local_offset = [sword_length, 0.0, 0.0] 
        
        # Apply Rotation: Global_Offset = Rotation_Matrix * Local_Offset
        dx = wrist_rotation[0] * local_offset[0] + wrist_rotation[1] * local_offset[1] + wrist_rotation[2] * local_offset[2]
        dy = wrist_rotation[3] * local_offset[0] + wrist_rotation[4] * local_offset[1] + wrist_rotation[5] * local_offset[2]
        dz = wrist_rotation[6] * local_offset[0] + wrist_rotation[7] * local_offset[1] + wrist_rotation[8] * local_offset[2]

        # Compute global sword tip position
        global_sword_tip = [
            wrist_position[0] + dx,
            wrist_position[1] + dy,
            wrist_position[2] + dz
        ]
        
        print(f"Global Sword Tip: {global_sword_tip}")
        return global_sword_tip 
    
    # Position tracking helpers
    def get_arm_wrist(self) -> List[float]:
            """Get the current global position of the arm's wrist."""
            wrist_node = self.combot.getFromDef("PLAYER_WRIST")
            wrist_position = wrist_node.getPosition()
            return wrist_position
        
    # IK chain helpers (delegates to ikpy_integration)
    def create_right_arm_chain(self):
        """Create IK chain for the right arm from URDF file."""
        return ik.create_right_arm_chain(self.urdf_file)