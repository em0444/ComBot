from controllers.tiagopp.combot import Combot

combot = Combot()

class Actions:
    def __init__(self, actionType, endPosition, complete):
        self.actionType = actionType
        self.endPosition = endPosition
        self.complete = complete


def decideMove():
    currentPosition = combot.get_position()
    currentArmPosition = combot.get_arm_position()
    currentSwordPosition = combot.get_sword_position()
    enemyPosition = combot.get_enemy_position()
    enemyArmPosition = combot.get_enemy_arm_position()
    enemySwordPosition = combot.get_enemy_sword_position()
    optimalMove = strategy1

    return optimalMove(currentPosition,currentArmPosition,currentSwordPosition,enemyPosition,enemyArmPosition,enemySwordPosition)

def distance(vec1,vec2):
    return ((vec1[0]-vec2[0])^2 + (vec1[1]-vec2[1])^2 + (vec1[2]-vec2[2])^2)^0.5

def strategy1(currentPosition,currentArmPosition,currentSwordPosition,enemyPosition,enemyArmPosition,enemySwordPosition):
    if distance(currentPosition,enemyPosition) < 1:
        return Actions("Attack",enemyPosition,False)
    return Actions("EnGarde",currentSwordPosition,True)