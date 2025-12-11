"""combot controller."""
import math
from typing import Callable, Dict

from controller import wb as c_webots_api, \
    Motor  # The wb package gives you all the C-like methods, but the controller package wraps most of them in nicer-to-use classes.
from combot import Combot
from shared_dataclasses import Position
from arm import Arm
import fencing_constants as fc
import fencing_actions as fence
import csv

# Shorthand alias for the Webots API module
wb = c_webots_api.wb

def train():
    import Qlearning
    import torch

    model_name = "temp"

    Qlearning.target_net.load_state_dict(torch.load("./DQN_states/model3f.pt", weights_only=True))
    Qlearning.target_net.eval()
    Qlearning.policy_net.load_state_dict(torch.load("./DQN_states/model3f.pt", weights_only=True))
    Qlearning.policy_net.eval()


    combot: Combot = Combot()
    combotNode = combot.getSelf()
    combotEnemy = combot.getFromDef("OPP")
    
    results = []
    
    timestep = int(combot.getBasicTimeStep())
    arm: Arm = Arm(combot, fc.RIGHT_ARM_CONFIG)
    fence.init(arm) # Initialise fencing module with arm reference
    

    combotNode.saveState("Init1")
    combotEnemy.saveState("Init2")
    for episode in range(0,Qlearning.num_episodes):
        ''' At the start of each episode, reset the robots positions and controllers and set data collection variables to defaults'''
        combotEnemy.restartController()
        combotNode.loadState("Init1")
        combotEnemy.loadState("Init2")
        combot.reset()
        state = Qlearning.getState(combot)
        stepSuccess = 0
        counter = 0
        rewardSum = 0
        endReason=""
        print("reset")
        previous = "STILL"
        combot.localisation = None
        while stepSuccess != -1:
            action = Qlearning.select_action(state)
            Qlearning.ACTIONSPACE[action-1]()#Preform actions

            if combot.base_state=="FORWARD":
                if previous == "BACKWARD":
                    combot.movement = None
                combot.move_to_position(Position(3.0, 0, 0), counter)
            elif combot.base_state=="BACKWARD":
                if previous == "FORWARD":
                    combot.movement = None
                combot.move_to_position(Position(-2.0, 0, 0), counter)
            counter+=1
            previous = combot.base_state

            stepSuccess = combot.step(timestep)

            observation = Qlearning.getState(combot)
            if counter < 2500:#End episode if it takes longer than a minute
                reward, terminated, endReason = Qlearning.getReward(combotNode,combotEnemy)
            else:
                reward = (torch.tensor([-1000], device=Qlearning.device),True)
                terminated = True
                endReason = "Ran out of time"
            rewardSum+=reward.item()

            if terminated:
                next_state = None
            else:
                next_state = observation

            # Store the transition in memory
            Qlearning.memory.push(state, action, next_state, reward)

            # Move to the next state
            state = next_state

            # Perform one step of the optimization (on the policy network)
            Qlearning.optimize_model()

            # Soft update of the target network's weights
            target_net_state_dict = Qlearning.target_net.state_dict()
            policy_net_state_dict = Qlearning.policy_net.state_dict()
            for key in policy_net_state_dict:
                target_net_state_dict[key] = policy_net_state_dict[key]*Qlearning.TAU + target_net_state_dict[key]*(1-Qlearning.TAU)
            Qlearning.target_net.load_state_dict(target_net_state_dict)

            if terminated:
                Qlearning.episode_durations.append(counter)
                break
        print(episode)
        results.append({'episode': episode, 'length': counter, 'end reason' : endReason , 'reward sum':rewardSum})

    with open("./DQN_training_results/"+model_name+".csv", "w") as f:
        writer = csv.DictWriter(f, fieldnames=['episode', 'length', 'end reason', 'reward sum'])
        writer.writeheader()
        writer.writerows(results)


    
    print('Complete')

    #print("Saving NN")
    #torch.save(Qlearning.target_net.state_dict(), "./DQN_states/"+model_name+".pt")

    Qlearning.plot_durations(show_result=True)
    Qlearning.plt.ioff()
    Qlearning.plt.show()

def botMain():
    import Qlearning
    import torch
    Qlearning.target_net.load_state_dict(torch.load("./DQN_states/model3f.pt", weights_only=True))
    Qlearning.target_net.eval()
    Qlearning.policy_net.load_state_dict(torch.load("./DQN_states/model3f.pt", weights_only=True))
    Qlearning.policy_net.eval()

    combot: Combot = Combot()
    timestep = int(combot.getBasicTimeStep())
    arm: Arm = Arm(combot, fc.RIGHT_ARM_CONFIG)
    fence.init(arm) # Initialise fencing module with arm reference
    stepSuccess = 0
    counter = 0
    while stepSuccess != -1:
        state = Qlearning.getState(combot)
        action = Qlearning.select_action(state)
        Qlearning.ACTIONSPACE[action-1]()#Preform actions

        if combot.base_state=="FORWARD":
            if previous == "BACKWARD":
                combot.movement = None
            combot.move_to_position(Position(3.0, 0, 0), counter)
        elif combot.base_state=="BACKWARD":
            if previous == "FORWARD":
                combot.movement = None
            combot.move_to_position(Position(-2.0, 0, 0), counter)
        counter +=1
        previous = combot.base_state

        stepSuccess = combot.step(timestep)

if __name__ == "__main__":
    if fc.ISTRAINING:
        train()
    elif fc.ISBOT:
        botMain()