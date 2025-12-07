# Arm controller wrapper — uses existing fencing_actions and ikpy_integration modules
import os
from typing import List, TYPE_CHECKING

import matplotlib
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
# from combot import Combot
import fencing_constants as fc
import fencing_actions as fence
import ikpy_integration as ik

if TYPE_CHECKING:
    from combot import Combot
class Arm:
    """High-level API to control combot arms using existing modules."""

    def __init__(self, combot: Combot, config: dict):
        self.combot = combot
        self.timestep = int(combot.getBasicTimeStep())
        self.config = config
        self.motors = {}
        self.sensors = {}

        self._init_all_sensors()
        self.urdf_file = self._check_urdf()
        self.right_arm_chain = self.create_right_arm_chain()
        self.opponent_node = self.combot.getFromDef("OPP")  # Tracks opponent sword base

    def _init_all_sensors(self):
        """Initialises every motor listed in FULL_BODY_PART_NAMES."""
        print("Enabling all joint sensors...")
        for motor_name in fc.FULL_BODY_PART_NAMES:
            try:
                motor = self.combot.getDevice(motor_name)
                self.motors[motor_name] = motor
                sensor = motor.getPositionSensor()
                if sensor: 
                    sensor.enable(self.timestep)
                    self.sensors[motor_name] = sensor
            except AttributeError:
                pass
    
        """Initialises the sword touch sensor."""
        print("Enabling sword touch sensor...")
        sword_sensor = self.combot.getDevice("sword_tip_sensor")
        sword_sensor.enable(self.timestep)
        self.sensors["sword_tip"] = sword_sensor
        print("Enabled Sword Tip Sensor")

    def _check_urdf(self):
        """Generates URDF if missing."""
        filename = "combot_urdf.urdf"
        if not os.path.exists(filename):
            print(f"URDF file not found. Generating {filename}...")
            with open(filename, "w") as file:
                file.write(self.combot.getUrdf())
        else:
            print(f"Found existing {filename}, skipping generation.")
    
        return filename

    # Sensor / state helpers
    def get_joint_angles(self) -> None:
        """Returns a dictionary of joint angles from all enabled sensors."""
        angles = {}
        for sensor in fc.SENSOR_NAMES:
            sensor = self.combot.getDevice(sensor)
            value = sensor.getValue()
            angles[sensor.name] = value
        
        print(angles)

    def get_right_joint_angles(self) -> dict:
        """Returns a dictionary of joint angles from all enabled sensors."""
        angles = {}
        
        for link in fc.RIGHT_SENSOR_NAMES:
            sensor = self.combot.getDevice(link)
            value = sensor.getValue()
            angles[sensor.name] = value
            
        print("Right Arm Angles:", angles)
        return angles

    # Direct pose / actions (delegates to fencing_actions)
    def move_to_pose(self, positions: List[float]) -> None:
        for part_name, position in zip(fc.FULL_BODY_PART_NAMES, positions):
            motor = self.motors[part_name]
            motor.setVelocity(motor.getVelocity())
            motor.setPosition(position)

    # Opponent targeting helper
    def get_opponent_target(self) -> List[float]:

        """
        Returns the [x, y, z] coordinates of the opponents center.
    
        Returns: list[float]: The global coordinates [x, y, z] of the opponent
        """
        self.opponent_node = self.combot.getFromDef("OPP")  # Tracks opponent sword base
        
        # if self.opponent_node is None:
        #     # Try finding it one last time (in case it spawned late)
        #     self.opponent_node = self.combot.getFromDef("OPP")
        # else:
        #     print("Error: Could not find opponent!")
        #     return [0, 0, 0]
        
        print("Opponent Anchor Node:", self.opponent_node)
        opponent_origin = self.opponent_node.getPosition()
        print("Opponent Origin:", opponent_origin)
        opponent_rotation = self.opponent_node.getOrientation() 
        print("Opponent Rotation Matrix:", opponent_rotation)
        
        # [3.77, -2.46776e-06, 0.085] gives a backwards parry
        # Define Fixed Local Offsets (Vector from Sword Handle -> Chest)
        # [Forward, Left/Right, Up/Down] relative to opponent center
        opponent_offset_local = [0.9, -0.4, -0.3] # TODO: depends on height of opponent
        # opponent_offset_local = [0, -1.45, 0.5] # relative to a shoulder pose
        # opponent_offset_local = [0, 0, 0] # relative to a centered pose
        
        # Extract matrix rows (Webots returns [r0c0, r0c1, r0c2, r1c0...])
        r0 = opponent_rotation[0:3] # Row 0 (X axis)
        r1 = opponent_rotation[3:6] # Row 1 (Y axis)
        r2 = opponent_rotation[6:9] # Row 2 (Z axis)
        
        # Dot product to apply rotation to the Offset (Matrix Multiplication)
        # Formula: Global_Offset = Rotation_Matrix * Local_Offset
        dx = r0[0]*opponent_offset_local[0] + r0[1]*opponent_offset_local[1] + r0[2]*opponent_offset_local[2]
        dy = r1[0]*opponent_offset_local[0] + r1[1]*opponent_offset_local[1] + r1[2]*opponent_offset_local[2]
        dz = r2[0]*opponent_offset_local[0] + r2[1]*opponent_offset_local[1] + r2[2]*opponent_offset_local[2]
        
        # Add Rotated Offset to Origin
        opponent_target_pos = [
            opponent_origin[0] + dx,
            opponent_origin[1] + dy,
            opponent_origin[2] + dz
        ]
        print("Opponent Global Position:", opponent_target_pos)
        return opponent_target_pos
    
    # IK Movement
    def move_to_target(self, target_pos, orientation_mode="Z", target_orientation=[0,0,1]) -> None:
        arm_angles = self.get_right_joint_angles()
        # I've got one disabled line at the front and four at the end
        current_angles = [0] + [angle for angle in arm_angles.values()] + [0]*4
        current_angles = current_angles[:len(self.right_arm_chain.links)]
        print("Initial Chain Position:", current_angles)

        # Solve arm IK to reach target position
        ikResults = self.right_arm_chain.inverse_kinematics(
            target_position=target_pos,  
            target_orientation = target_orientation, # direction vector pointing "up"
            orientation_mode=orientation_mode,
            initial_position=current_angles # Starting position for IK solver; optimatisation speedup 
            )
        
        print("IK Results:", ikResults)

        for res in range(len(ikResults)):
            # This if check will ignore anything that isn't controllable
            if self.right_arm_chain.links[res].name in fc.FULL_BODY_PART_NAMES:
                self.combot.getDevice(self.right_arm_chain.links[res].name).setPosition(ikResults[res])
                print("Setting {} to {}".format(self.right_arm_chain.links[res].name, ikResults[res]))
        
        print("Moved arm to target position:", target_pos)
        # Visualize the IK results 
        # Red dot indicates the exact point in space we're trying to reach
        ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

        # Plot the current position of your arm in blue
        self.right_arm_chain.plot(current_angles, ax, target=target_pos)

        # And plot the target position of your arm in orange
        self.right_arm_chain.plot(ikResults, ax, target=target_pos)
        matplotlib.pyplot.show()

    # IK helpers (delegates to ikpy_integration)
    def create_right_arm_chain(self):
        """Create and activate an arm chain for the right arm."""
        return ik.create_right_arm_chain(self.urdf_file)

    def initialise_ikpy_integration(self):
        """Run the existing IK routine (moves the arm toward opponent)."""
        return ik.initialise_ikpy_integration(self.right_arm_chain)
    
    # For strategy
    def get_arm_wrist(self):
        our_fencer = self.combot.getFromDef("FENCER")
        print("Me:", our_fencer.getPosition())
        our_opp = self.combot.getFromDef("OPP")
        # print("Them:", our_opp.getPosition())
        my_sword = self.combot.getDevice("sword_tip_sensor")
        sword_tip = self.get_real_sword_tip()
        print("Sword tip: ", sword_tip)
        my_wrist = self.combot.getFromDef("PLAYER_WRIST")
        wrist_pos = my_wrist.getPosition()
        return wrist_pos
    
    def get_real_sword_tip(self):
        """
        Calculates the global sword tip position using the Supervisor API.
        This is more accurate than FK because it accounts for the robot's 
        movements and body rotation.
        """
        # 1. Get the Wrist Node (defined in Tiago++.proto)
        wrist_node = self.combot.getFromDef("PLAYER_WRIST")
        
        if wrist_node is None:
            print("Error: PLAYER_WRIST node not found!")
            return [0, 0, 0]

        # 2. Get Global Position and Rotation of the Wrist
        wrist_pos = wrist_node.getPosition()
        wrist_rot = wrist_node.getOrientation() # Returns 3x3 Matrix as list of 9 floats

        # 3. Define the Sword Offset (Vector from Wrist -> Tip)
        # You must adjust this based on your specific sword model geometry.
        # Example: If sword extends 0.9m along the X-axis of the wrist:
        sword_length = 0.9 
        local_offset = [sword_length, 0.0, 0.0] 
        
        # 4. Apply Rotation to the Offset (Matrix Multiplication)
        # Global_Offset = Rotation_Matrix * Local_Offset
        # Matrix is flat: [r0, r1, r2, r3, r4, r5, r6, r7, r8]
        dx = wrist_rot[0] * local_offset[0] + wrist_rot[1] * local_offset[1] + wrist_rot[2] * local_offset[2]
        dy = wrist_rot[3] * local_offset[0] + wrist_rot[4] * local_offset[1] + wrist_rot[5] * local_offset[2]
        dz = wrist_rot[6] * local_offset[0] + wrist_rot[7] * local_offset[1] + wrist_rot[8] * local_offset[2]

        # 5. Add Rotated Offset to Wrist Position
        global_tip = [
            wrist_pos[0] + dx,
            wrist_pos[1] + dy,
            wrist_pos[2] + dz
        ]
        print("-" * 30)
        print(f"OFFSET CALCULATION:")
        print(f"Forward (X): {global_tip[0]:.4f} m")
        print(f"Left    (Y): {global_tip[1]:.4f} m")
        print(f"Up      (Z): {global_tip[2]:.4f} m")
        print("-" * 30)
        
        # print(f"Global Sword Tip: {global_tip}")
        return global_tip
    
    def get_sword_tip_position(self):
        """
        Returns the [x, y, z] position of the sword tip 
        relative to the Right Shoulder (the base of the chain).
        """
        # 1. Get current joint angles (same as you do for IK)
        arm_angles = self.get_right_joint_angles()
        
        # 2. Pad the list to match the chain (Base + 7 Joints + Tip Padding)
        # We perform the same list construction you use in move_to_target
        current_angles = [0] + [angle for angle in arm_angles.values()] + [0]*4
        
        # Slice to ensure it fits the chain length exactly
        current_angles = current_angles[:len(self.right_arm_chain.links)]

        # 3. Calculate Forward Kinematics
        # This returns a 4x4 Transformation Matrix
        transformation_matrix = self.right_arm_chain.forward_kinematics(current_angles)
        
        # 4. Extract the Position Vector (First 3 rows of the last column)
        # Matrix structure:
        # [ r00 r01 r02  X ]
        # [ r10 r11 r12  Y ]
        # [ r20 r21 r22  Z ]
        # [  0   0   0   1 ]
        x = transformation_matrix[0, 3]
        y = transformation_matrix[1, 3]
        z = transformation_matrix[2, 3]
        
        print(f"Sword Tip (Local): [{x:.2f}, {y:.2f}, {z:.2f}]")
        return [x, y, z]
    
    def get_sword_tip_global(self):
        # 1. Get Local Tip Position
        local_tip = self.get_sword_tip_position()
        
        # 2. Get Robot's Global Position (Requires Supervisor)
        robot_pos = self.combot.getSelf().getPosition()
        
        # 3. Add them together
        # (Note: This is a simplification; for perfect accuracy you would 
        # need to apply the robot's body rotation matrix too)
        global_tip = [
            robot_pos[0] + local_tip[0],
            robot_pos[1] + local_tip[1],
            robot_pos[2] + local_tip[2]
        ]
        print("Global tip pos: ", global_tip)
        return global_tip