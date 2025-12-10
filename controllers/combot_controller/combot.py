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
    previousPosition = Position(-2.0,0.0,0.0)
    previouspreviousPosition = Position(-2.0,0.0,0.0)
    previousVelocityX = 0.0
    enemyPosition = Position(2.0,0.0,math.pi)
    previousEnemyPosition = Position(2.0,0.0,math.pi)
    previouspreviousEnemyPosition = Position(2.0,0.0,math.pi)
    previousEnemyVelocityX = 0.0

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
        self.previousVelocityX = self.previousPosition.x-self.previouspreviousPosition.x
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
        return (self.get_x_velocity()-self.previousVelocityX)
    

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
        self.previousEnemyVelocityX = self.previousEnemyPosition.x-self.previouspreviousEnemyPosition.x
        self.enemyPosition = self.localisation.get_enemy_position()
        if self.enemyPosition is None:
            self.enemyPosition = Position((self.getFromDef("OPP").getPosition()[0]),0.0,-math.pi)
            #print("Lidar not loaded")
        return self.enemyPosition
    
    def get_enemy_x_velocity(self):
        return self.enemyPosition.x-self.previousEnemyPosition.x
    
    def get_enemy_x_acceleration(self):
        return self.get_enemy_x_velocity()-self.previousEnemyVelocityX
    
    def get_enemy_arm_position(self):
        raise NotImplementedError()
    
    def get_enemy_sword_position(self):
        raise NotImplementedError()
    
    def reset(self):
        self.changing_body_state = False
        self.body_state = "DEFAULT"
        self.changing_base_state = False
        self.base_state = "STILL"
        self.timeCounter = 0.0
        self.cycleIndex = -1
        self.previousPosition = Position(-2.0,0.0,0.0)
        self.previouspreviousPosition = Position(-2.0,0.0,0.0)
        self.previousVelocityX = 0.0
        self.enemyPosition = Position(2.0,0.0,math.pi)
        self.previousEnemyPosition = Position(2.0,0.0,math.pi)
        self.previouspreviousEnemyPosition = Position(2.0,0.0,math.pi)
        self.previousEnemyVelocityX = 0.0

    # def start_timer(self):
    #     self.start_timestamp = self.getTime()

    # def get_elapsed_time(self):
    #     return self.getTime() - self.start_timestamp