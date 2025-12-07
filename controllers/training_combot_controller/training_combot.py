from typing import List

from controller import Supervisor


class TrainerCombot(Supervisor):
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
        self._initialized = True

    def get_position(self):
        return self.getSelf().getPosition()
    
    def get_arm_wrist_position(self):
        raise NotImplementedError()
    
    def get_sword_tip_position(self):
        raise NotImplementedError()
    
    def get_enemy_position(self):
        return self.getFromDef("ComBot").getPosition()
    
    def get_enemy_arm_position(self):
        raise NotImplementedError()
    
    def get_enemy_sword_position(self):
        raise NotImplementedError()