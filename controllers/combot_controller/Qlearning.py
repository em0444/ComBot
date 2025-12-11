import math
import random
import matplotlib
import matplotlib.pyplot as plt
from collections import namedtuple, deque
from itertools import count

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

from fencing_actions import en_garde,lunge,parry_high,parry_low,move_forward,move_backward,move_stop,check_hit
from combot import Combot
from shared_dataclasses import Position

ACTIONSPACE = [en_garde,lunge,parry_high,parry_low,move_forward,move_backward,move_stop]
ALLOBSERVATIONS = [Combot.get_position,Combot.get_x_velocity,Combot.get_x_acceleration,Combot.get_enemy_position,Combot.get_enemy_x_velocity,Combot.get_enemy_x_acceleration,1.0 if Combot.body_state=="EN_GARDE" else 0.0,1.0 if Combot.body_state=="LUNGE" else 0.0,1.0 if Combot.body_state=="PARRY_HIGH" else 0.0,1.0 if Combot.body_state=="PARRY_LOW" else 0.0,1.0 if Combot.base_state=="FORWARD" else 0.0,1.0 if Combot.base_state=="BACKWARD" else 0.0,1.0 if Combot.base_state=="STILL" else 0.0]

plt.ion()

# if GPU is to be used
device = torch.device(
    "cuda" if torch.cuda.is_available() else
    "mps" if torch.backends.mps.is_available() else
    "cpu"
)



Transition = namedtuple('Transition',
                        ('state', 'action', 'next_state', 'reward'))


class ReplayMemory(object):

    def __init__(self, capacity):
        self.memory = deque([], maxlen=capacity)

    def push(self, *args):
        """Save a transition"""
        self.memory.append(Transition(*args))

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)
    
class DeepQNetwork(nn.Module):

    def __init__(self, n_observations, n_actions):
        super(DeepQNetwork, self).__init__()
        self.layer1 = nn.Linear(n_observations, 128)
        self.layer2 = nn.Linear(128, 128)
        self.layer3 = nn.Linear(128, n_actions)

    # Called with either one element to determine next action, or a batch
    # during optimization. Returns tensor([[left0exp,right0exp]...]).
    def forward(self, x):
        x = F.relu(self.layer1(x))
        x = F.relu(self.layer2(x))
        return self.layer3(x)
    
def getState(robot:Combot):
    x = robot.get_position()
    if x is None:
        print("x position not found.")
        x =robot.getSelf().getPosition()[0]
    else:
        x = x.x

    try:
        xenemy = robot.get_enemy_position()
    except Exception:
        xenemy = None
    if xenemy is None:
        print("x position not found.")
        xenemy =robot.getFromDef("OPP").getPosition()[0]
    else:
        xenemy = xenemy.x

    return torch.tensor([x,robot.get_x_velocity(),robot.get_x_acceleration(),xenemy,robot.get_enemy_x_velocity(),robot.get_enemy_x_acceleration(),1.0 if robot.body_state=="EN_GARDE" else 0.0,1.0 if robot.body_state=="LUNGE" else 0.0,1.0 if robot.body_state=="PARRY_HIGH" else 0.0,1.0 if robot.body_state=="PARRY_LOW" else 0.0,1.0 if robot.base_state=="FORWARD" else 0.0,1.0 if robot.base_state=="BACKWARD" else 0.0,1.0 if robot.base_state=="STILL" else 0.0], dtype=torch.float32, device=device).unsqueeze(0)


#Hyper parameters
BATCH_SIZE = 128
GAMMA = 0.99
EPS_START = 0.01
EPS_END = 0.01
EPS_DECAY = 2500
TAU = 0.005
LR = 3e-4


# Get number of actions from gym action space
n_actions = len(ACTIONSPACE)
# Get the number of state observations
#reset the environment
n_observations = len(ALLOBSERVATIONS)

policy_net = DeepQNetwork(n_observations, n_actions).to(device)
target_net = DeepQNetwork(n_observations, n_actions).to(device)
target_net.load_state_dict(policy_net.state_dict())

optimizer = optim.AdamW(policy_net.parameters(), lr=LR, amsgrad=True)
memory = ReplayMemory(10000)


steps_done = 0


def select_action(state):
    global steps_done
    sample = random.random()
    eps_threshold = EPS_END + (EPS_START - EPS_END) * \
        math.exp(-1. * steps_done / EPS_DECAY)
    steps_done += 1
    if sample > eps_threshold:
        with torch.no_grad():
            # t.max(1) will return the largest column value of each row.
            # second column on max result is index of where max element was
            # found, so we pick action with the larger expected reward.
            return policy_net(state).max(1).indices.view(1, 1)
    else:
        return torch.tensor([[random.randint(1,7)]], device=device, dtype=torch.long)


episode_durations = []


def plot_durations(show_result=False):
    plt.figure(1)
    durations_t = torch.tensor(episode_durations, dtype=torch.float)
    if show_result:
        plt.title('Result')
        plt.savefig("Results.png")
    else:
        plt.clf()
        plt.title('Training...')
    plt.xlabel('Episode')
    plt.ylabel('Duration')
    plt.plot(durations_t.numpy())
    # Take 100 episode averages and plot them too
    if len(durations_t) >= 100:
        means = durations_t.unfold(0, 100, 1).mean(1).view(-1)
        means = torch.cat((torch.zeros(99), means))
        plt.plot(means.numpy())

    plt.pause(0.001)  # pause a bit so that plots are updated




def optimize_model():
    if len(memory) < BATCH_SIZE:
        return
    transitions = memory.sample(BATCH_SIZE)

    batch = Transition(*zip(*transitions))

    non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
                                          batch.next_state)), device=device, dtype=torch.bool)
    non_final_next_states = torch.cat([s for s in batch.next_state
                                                if s is not None])
    state_batch = torch.cat(batch.state)
    action_batch = torch.cat(batch.action)
    reward_batch = torch.cat(batch.reward)

    state_action_values = policy_net(state_batch).gather(1, action_batch)

    next_state_values = torch.zeros(BATCH_SIZE, device=device)
    with torch.no_grad():
        next_state_values[non_final_mask] = target_net(non_final_next_states).max(1).values
    # Compute the expected Q values
    expected_state_action_values = (next_state_values * GAMMA) + reward_batch

    # Compute Huber loss
    criterion = nn.SmoothL1Loss()
    loss = criterion(state_action_values, expected_state_action_values.unsqueeze(1))

    # Optimize the model
    optimizer.zero_grad()
    loss.backward()
    # In-place gradient clipping
    torch.nn.utils.clip_grad_value_(policy_net.parameters(), 100)
    optimizer.step()

if torch.cuda.is_available() or torch.backends.mps.is_available():
    num_episodes = 100
else:
    num_episodes = 50

def isOutsideBounds(pos):
    if pos[1]<-0.35 or pos[1]>0.35:
        return True
    if pos[0]<-2.5 or pos[0]>4.2:
        return True
    if pos[2]<-0.1 or pos[2]>10.2:
        return True
    return False

def hasFallen(rotation):#Check rotation vectors aren't too high or low
    if rotation[6]<-0.15 or rotation[6]>0.15 or rotation[7]<-0.15 or rotation[7]>0.15:
        return True
    return False

def distance(pos1,pos2):
    return abs(pos1[0]-pos2[0])

def getReward(combotNode,trainerNode):
    if (isOutsideBounds(combotNode.getPosition())):
        print("Combot out of bounds")
        return (torch.tensor([-200], device=device),True,"Fencer went out of bounds")
    if (isOutsideBounds(trainerNode.getPosition())):
        print("Trainer out of bounds")
        return (torch.tensor([0], device=device),True,"Opponent went out of bounds")
    if (hasFallen(combotNode.getOrientation())):
        print("Combot fallen over")
        return (torch.tensor([-100], device=device),True,"Fencer fell over")
    if (hasFallen(trainerNode.getOrientation())):
        print("Trainer fallen over")
        return (torch.tensor([200], device=device),True,"Opponent fell over")
    if distance(combotNode.getPosition(),trainerNode.getPosition())<1:
        print("Too close")
        return (torch.tensor([-50], device=device),True,"Fencer got to close to opponent")


    if check_hit():
        if distance(combotNode.getPosition(),trainerNode.getPosition())<1.8:
            print("Opponent body hit")
            return (torch.tensor([200], device=device),True,"Fencer hit opponent")
    
    if (distance(combotNode.getPosition(),trainerNode.getPosition())>3):
        return (torch.tensor([-0.5], device=device),False,None)
    return (torch.tensor([0], device=device),False,None)