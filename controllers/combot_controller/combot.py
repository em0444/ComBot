import math
from typing import List, Tuple

from controller import Supervisor, Robot
from shared_dataclasses import Position

class Combot(Supervisor):
    _instance = None
    changing_body_state = False
    body_state = "DEFAULT"
    changing_base_state = False
    base_state = "STILL"
    timeCounter = 0.0
    cycleIndex = -1
    previousPosition = -2.0
    previouspreviousPosition = -2.0
    previousVelocity = 0.0
    enemyPosition = 2.0
    previousEnemyPosition = 2.0
    previouspreviousEnemyPosition = 2.0
    previousEnemyVelocity = 0.0

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
        self.movement = None
        self._initialized = True
        self.position = Position(0, 0, 0)
        # self.arm = Arm(self, config=fc.RIGHT_ARM_CONFIG)

    def update_internal_position_model(self) -> None:
        if self.localisation is None:
            from localisation import Localisation
            self.localisation = Localisation(combot_obj=self)
        self.previouspreviousPosition = self.previousPosition
        self.previousPosition = self.position
        self.previousVelocity = self.previousPosition-self.previouspreviousPosition
        self.position = self.localisation.update_internal_position_model()

    def get_position(self) -> Position:
        if self.localisation is None:
            from localisation import Localisation
            self.localisation = Localisation(combot_obj=self)
        return self.position
    
    def get_x_velocity(self):
        return (self.position.x-self.previousPosition.x)
    
    def get_z_velocity(self):
        return (self.position.x-self.previousPosition.x)
    
    def get_angular_velocity(self):
        return (self.position.heading_in_radians-self.previousPosition.heading_in_radians)
    
    def get_x_acceleration(self):
        return (self.get_x_velocity-self.previousVelocity)
    

    def move_to_position(self, position: Position, counter: int) -> bool:
        """
        A non-blocking function to move the robot, while simultaneously allowing it to do other things.
        Returns True if the robot has successfully completed the manouvre.
        If it returns false, increment a timestep, increment the counter by one, and call it again.
        """
        if self.localisation is None:
            from localisation import Localisation
            self.localisation = Localisation(combot_obj=self)
        if self.movement is None or counter == 0:
            from movement import Movement
            self.movement = Movement(combot=self, target_position=position)
        return self.movement.move_to_position(counter=counter)

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
        self.previouspreviousEnemyPosition = self.previousEnemyPosition
        self.previousEnemyPosition = self.enemyPosition
        self.previousEnemyVelocity = self.previousEnemyPosition-self.previouspreviousEnemyPosition
        self.enemyPosition = self.localisation.get_enemy_position()
        return self.enemyPosition
    
    def get_enemy_x_velocity(self):
        return self.enemyPosition-self.previousEnemyPosition
    
    def get_enemy_x_acceleration(self):
        return self.get_enemy_x_velocity()-self.previousEnemyVelocity
    
    def get_enemy_arm_position(self):
        raise NotImplementedError()
    
    def get_enemy_sword_position(self):
        raise NotImplementedError()

    # def sword_is_contacting(self):
    #     supervisor_contact_points = self.supervisor_obj.getFromDef("FENCING_SWORD_SOLID").getContactPoints()
    #     return len(supervisor_contact_points) > 0 # If you want this to be less sensitive in future, set this = 1