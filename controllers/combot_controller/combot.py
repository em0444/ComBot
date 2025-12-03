from typing import List

from controller import Supervisor


class Combot(Supervisor):
    _instance = None
    changing_state = False
    current_state = "DEFAULT"
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

    def get_position(self) -> List[float]:
        from localisation import get_position
        return get_position()
    
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
    
    # def sword_is_contacting(self):
    #     supervisor_contact_points = self.supervisor_obj.getFromDef("FENCING_SWORD_SOLID").getContactPoints()
    #     return len(supervisor_contact_points) > 0 # If you want this to be less sensitive in future, set this = 1