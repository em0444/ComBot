from typing import List, Tuple

from controller import Robot


class Combot(Robot):
    _instance = None
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

    def get_position(self) -> Tuple[float, float]:
        if self.localisation is None:
            from controllers.combot_controller.localisation import Localisation
            self.localisation = Localisation(combot_obj=self)
        return self.localisation.get_position()
    
    def get_arm_position(self):
        raise NotImplementedError()
    
    def get_sword_position(self):
        raise NotImplementedError()
    
    def get_enemy_position(self):
        raise NotImplementedError()
    
    def get_enemy_arm_position(self):
        raise NotImplementedError()
    
    def get_enemy_sword_position(self):
        raise NotImplementedError()