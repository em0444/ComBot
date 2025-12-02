from combot import Combot
from fencing_actions import lunge, parry_high, parry_low, en_garde

HITRANGE = 1.0

combot = Combot()


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

def strategy0(currentPosition,currentArmPosition,currentSwordPosition,enemyPosition,enemyArmPosition,enemySwordPosition):
    return None

def strategy1(currentPosition,currentArmPosition,currentSwordPosition,enemyPosition,enemyArmPosition,enemySwordPosition):
    if not combot.changing_state and combot.current_state != "EN_GARDE":
        return en_garde
    return None

def strategy2(currentPosition,currentArmPosition,currentSwordPosition,enemyPosition,enemyArmPosition,enemySwordPosition):
    if not combot.changing_state and combot.current_state != "LUNGE":
        return lunge
    return None

def strategy3(currentPosition,currentArmPosition,currentSwordPosition,enemyPosition,enemyArmPosition,enemySwordPosition):
    if not combot.changing_state:
        if distance(currentPosition,enemyPosition) < HITRANGE and combot.current_state != "LUNGE":
            return lunge
        if combot.current_state != "EN_GARDE":
            return en_garde
    return None

def strategy4(currentPosition,currentArmPosition,currentSwordPosition,enemyPosition,enemyArmPosition,enemySwordPosition):
    if not combot.changing_state:
        if distance(enemySwordPosition,enemyPosition) > distance(enemySwordPosition,currentPosition):
            if combot.current_state == "PARRY_HIGH":
                return lunge
            return parry_high
        if combot.current_state != "EN_GARDE":
            return en_garde
    return None