from combot import Combot
from fencing_actions import lunge, parry_high, parry_low, en_garde
import random

HITRANGE = 1.0
ALLACTIONS =  [lunge, parry_high,parry_low,en_garde]
STRATEGY5ACTIONDELAY = 2000

combot = Combot()
timestep = int(combot.getBasicTimeStep())
timeEllapsed = 0

def decideMove():
    optimalMove = strategy5
    return optimalMove(combot)

def distance(vec1,vec2):
    return ((vec1[0]-vec2[0])^2 + (vec1[1]-vec2[1])^2 + (vec1[2]-vec2[2])^2)^0.5

def strategy0(robot):
    return None

def strategy1(robot):
    if not robot.changing_state and robot.current_state != "EN_GARDE":
        return en_garde
    return None

def strategy2(robot):
    if not robot.changing_state and robot.current_state != "LUNGE":
        return lunge
    return None

def strategy3(robot):
    if not robot.changing_state:
        if distance(robot.currentPosition,robot.enemyPosition) < HITRANGE and robot.current_state != "LUNGE":
            return lunge
        if robot.current_state != "EN_GARDE":
            return en_garde
    return None

def strategy4(robot):
    if not robot.changing_state:
        if distance(robot.enemySwordPosition,robot.enemyPosition) > distance(robot.enemySwordPosition,robot.currentPosition):
            if robot.current_state == "PARRY_HIGH":
                return lunge
            return parry_high
        if robot.current_state != "EN_GARDE":
            return en_garde
    return None

def templateStrategyCycle(actionList,timeInterval):
    def newStrategy(robot):
        if not robot.changing_state and robot.timeCounter<=robot.getTime():
            robot.timeCounter = robot.timeCounter + timeInterval
            robot.cycleIndex = robot.cycleIndex + 1
            if robot.cycleIndex >= len(actionList):
                robot.cycleIndex = 0
            return actionList[robot.cycleIndex]
        return None
    return newStrategy

def templateStrategyRandom(actionList,weights,timeInterval):
    if len(actionList)!=len(weights):
        return strategy0
    def newStrategy(robot):
        if not robot.changing_state and robot.timeCounter<=robot.getTime():
            robot.timeCounter = robot.timeCounter + timeInterval
            weightIndex = random.randint(1,sum(weights))
            for i in range(0,len(actionList)):
                if weights[i] < weightIndex:
                    weightIndex = weightIndex  - weights[i]
                else:
                    return actionList[i]
        return None
    return newStrategy

strategy5 = templateStrategyRandom(ALLACTIONS,[1,1,1,1],2.0)

strategy6 = templateStrategyRandom(ALLACTIONS,[1,1,1,1],1.0)