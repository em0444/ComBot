# Arm controller wrapper — uses existing fencing_actions and ikpy_integration modules
import os
from typing import List
from combot import Combot
import fencing_constants as fc
import fencing_actions as fence
import ikpy_integration as ik

class ArmController:
    """High-level API to control combot arms using existing modules."""

    def __init__(self, combot: Combot, config: dict):
        self.combot = combot
        self.timestep = int(combot.getBasicTimeStep())
        self.config = config
        self.motors = {}
        self.sensors = {}

        self._init_all_sensors()
        self._init_sword_sensor()
        self.urdf_file = self._check_urdf()
        self.right_arm_chain = self.create_right_arm_chain()

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
    def _init_sword_sensor(self):
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

    # Direct pose / actions (delegates to fencing_actions)
    def move_to_pose(self, positions: List[float]) -> None:
        for part_name, position in zip(fc.FULL_BODY_PART_NAMES, positions):
            motor = self.motors[part_name]
            motor.setVelocity(motor.getVelocity())
            motor.setPosition(position)

    # IK helpers (delegates to ikpy_integration)
    def create_right_arm_chain(self):
        """Create and activate an arm chain for the right arm."""
        return ik.create_right_arm_chain(self.urdf_file)

    def initialise_ikpy_integration(self):
        """Run the existing IK routine (moves the arm toward opponent)."""
        return ik.initialise_ikpy_integration(self.right_arm_chain)

    def get_right_joint_angles(self):
        """Get right arm joint angles via ikpy_integration helper."""
        return ik.get_right_joint_angles()