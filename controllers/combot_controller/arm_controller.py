# Arm controller wrapper — uses existing fencing_actions and ikpy_integration modules

from typing import Optional, List
from combot import Combot
import fencing_actions as fence
import ikpy_integration as ik

class ArmController:
    """High-level API to control combot arms using existing modules."""

    def __init__(self, combot_obj: Optional[Combot] = None):
        self.combot = combot_obj or Combot()

    # Sensor / state helpers
    def enable_sensors(self) -> None:
        """Enable all joint sensors (calls existing implementation)."""
        fence.enable_sensors()

    def get_joint_angles(self) -> None:
        """Print and return current joint angles (delegates to fencing_actions)."""
        return fence.get_joint_angles()

    # Direct pose / actions (delegates to fencing_actions)
    def move_to_pose(self, positions: List[float]) -> None:
        fence.move_to_pose(positions)

    def en_garde(self) -> None:
        fence.en_garde()

    def lunge(self) -> None:
        fence.lunge()

    def parry_high(self) -> None:
        fence.parry_high()

    def parry_low(self) -> None:
        fence.parry_low()

    # Sword / environment helpers
    def get_sword_handle_position(self):
        return fence.get_sword_handle_position()

    # IK helpers (delegates to ikpy_integration)
    def create_ik_chain(self):
        """Create and initialise an IK chain for the right arm."""
        return ik.create_ik_chain()

    def initialise_ikpy_integration(self):
        """Run the existing IK routine (moves the arm toward opponent)."""
        return ik.initialise_ikpy_integration()

    def get_right_joint_angles(self):
        """Get right arm joint angles via ikpy_integration helper."""
        return ik.get_right_joint_angles()