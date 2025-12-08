import math
from typing import List, Tuple

from controller import Supervisor, Robot
from shared_dataclasses import Position
# from arm import Arm
# import fencing_constants as fc

class Combot(Supervisor):
    _instance = None
    changing_body_state = False
    body_state = "DEFAULT"
    changing_base_state = False
    base_state = "STILL"
    timeCounter = 0.0
    cycleIndex = -1

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        super().__init__()
        self.localisation = None
        self._initialized = True
        self.position = Position(0, 0, 0)
        # self.arm = Arm(self, config=fc.RIGHT_ARM_CONFIG)

    def update_internal_position_model(self) -> None:
        if self.localisation is None:
            from localisation import Localisation
            self.localisation = Localisation(combot_obj=self)
        self.position = self.localisation.update_internal_position_model()

    def get_position(self) -> Position:
        return self.position

    def move_to_position(self, position: Position) -> None:
        from movement import move_to_position
        move_to_position(combot_obj=self, target_pos=position)

    def get_arm_wrist_position(self): 
        # return self.arm.get_arm_wrist()
        raise NotImplementedError()
    
    def get_sword_tip_position(self):
        # return self.arm.get_real_sword_tip()
        raise NotImplementedError()
    
    def get_enemy_position(self):
        if self.localisation is None:
            from localisation import Localisation
            self.localisation = Localisation(combot_obj=self)
        self.position = self.localisation.get_enemy_position()

        raise NotImplementedError()
    
    def get_enemy_arm_position(self):
        raise NotImplementedError()
    
    def get_enemy_sword_position(self):
        raise NotImplementedError()
    
    # def sword_is_contacting(self):
    #     supervisor_contact_points = self.supervisor_obj.getFromDef("FENCING_SWORD_SOLID").getContactPoints()
    #     return len(supervisor_contact_points) > 0 # If you want this to be less sensitive in future, set this = 1