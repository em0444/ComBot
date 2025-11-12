from controllers.tiagopp.combot import Combot

def decideMove():
    currentPosition = Combot.get_position()
    currentArmPosition = Combot.get_arm_position()
    currentSwordPosition = Combot.get_sword_position()
    enemeyPosition = Combot.get_enemey_position()
    enemeyArmPosition = Combot.get_enemey_arm_position()
    enemeySwordPosition = Combot.get_enemey_sword_position()
    optimalMove = strategy1

    return optimalMove(currentPosition,currentArmPosition,currentSwordPosition,enemeyPosition,enemeyArmPosition,enemeySwordPosition)


def strategy1():
    return None