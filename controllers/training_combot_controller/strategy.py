from trainer_fencing_actions import lunge, parry_high, parry_low, en_garde, move_backward, move_forward, move_stop
import random
from threading import Thread
from math import sqrt

HITRANGE = 1.0
ALLBODYACTIONS =  [lunge, parry_high,parry_low,en_garde]
STRATEGY5ACTIONDELAY = 2000

timeEllapsed = 0

def mergeActions(action1,action2):
    def executeBothActions():
        first = Thread(target = action1)
        second = Thread(target = action2)
        first.start()
        second.start()
    return executeBothActions

def mergeStrategies(strat1,strat2):
    def newMergedStrategy(robot):
        baseMove = strat1(robot)
        bodyMove = strat2(robot)
        if baseMove is None:
            if bodyMove is None:
                return None
            return bodyMove
        if bodyMove is None:
            return baseMove
        return mergeActions(baseMove,bodyMove)
    return newMergedStrategy

def distance(vec1,vec2):
    return sqrt((vec1[0]-vec2[0])**2 + (vec1[1]-vec2[1])**2 + (vec1[2]-vec2[2])**2)

def strategy0(robot):
    return None

def strategy1(robot):
    if not robot.changing_body_state and robot.body_state != "EN_GARDE":
        return en_garde
    return None

def strategy2(robot):
    if not robot.changing_body_state and robot.body_state != "LUNGE":
        return lunge
    return None

def strategy3(robot):
    if not robot.changing_body_state:
        if distance(robot.currentPosition,robot.enemyPosition) < HITRANGE and robot.body_state != "LUNGE":
            return lunge
        if robot.body_state != "EN_GARDE":
            return en_garde
    return None

def strategy4(robot):
    if not robot.changing_body_state:
        if distance(robot.enemySwordPosition,robot.enemyPosition) > distance(robot.enemySwordPosition,robot.currentPosition):
            if robot.body_state == "PARRY_HIGH":
                return lunge
            return parry_high
        if robot.body_state != "EN_GARDE":
            return en_garde
    return None

def templateStrategyCycle(actionList,timeInterval):
    def newCycleStrategy(robot):
        if robot.timeCounter<=robot.getTime():
            robot.timeCounter = robot.timeCounter + timeInterval
            robot.cycleIndex = robot.cycleIndex + 1
            if robot.cycleIndex >= len(actionList):
                robot.cycleIndex = 0
            return actionList[robot.cycleIndex]
        return None
    return newCycleStrategy

def templateStrategyRandom(actionList,weights,timeInterval):
    if len(actionList)!=len(weights):
        return strategy0
    def newStrategy(robot):
        if robot.timeCounter<=robot.getTime():
            robot.timeCounter = robot.timeCounter + timeInterval
            weightIndex = random.randint(1,sum(weights))
            for i in range(0,len(actionList)):
                if weights[i] < weightIndex:
                    weightIndex = weightIndex  - weights[i]
                else:
                    return actionList[i]
        return None
    return newStrategy

strategy5 = templateStrategyRandom(ALLBODYACTIONS,[1,1,1,1],2.0)

strategy6 = templateStrategyRandom(ALLBODYACTIONS,[1,1,1,1],1.0)

strategy7 = templateStrategyCycle([parry_high,parry_low],1.5)

def moveForwardStrategy(robot):
    if not robot.changing_base_state and robot.base_state != "FORWARD":
        return move_forward
    return None

def moveBackwardStrategy(robot):
    if not robot.changing_base_state and robot.base_state != "BACKWARD":
        return move_backward
    return None

strategy8 = mergeStrategies(strategy1,moveForwardStrategy)

strategy9 = mergeStrategies(strategy2,moveForwardStrategy)

strategy10 = templateStrategyCycle([mergeActions(move_forward,lunge),move_stop,mergeActions(move_backward,en_garde),move_stop],1.5)

def strategy11(robot):
    if not robot.changing_body_state:
        if distance(robot.currentPosition,robot.enemyPosition) < HITRANGE and robot.body_state != "LUNGE":
            return mergeActions(lunge,move_forward)
        if robot.body_state != "EN_GARDE":
            return en_garde
    return None

strategy12 = templateStrategyRandom([move_forward,move_backward,move_stop,en_garde,lunge,parry_high,parry_low],[1,1,1,1,1,1,1],1.0)
