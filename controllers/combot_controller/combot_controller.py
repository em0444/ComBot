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
import strategy as strat
import csv

ISUSER =False
ISTRAINING = True

# Movement Keyboard codes (Webots specific)
KEY_UP = 315
KEY_DOWN = 317
KEY_RIGHT = 316
KEY_LEFT = 314

# Fencing Action Keys
KEY_LUNGE = 32      # Spacebar
KEY_PARRY_HIGH = 81 # Q
KEY_PARRY_LOW = 90  # Z
KEY_EN_GARDE = 82   # R

# Shorthand alias for the Webots API module
wb = c_webots_api.wb

def handle_movement_speed(key, max_speed):
    speed_left = 0.0
    speed_right = 0.0

    if key == KEY_UP: 
        speed_left, speed_right = max_speed, max_speed
    elif key == KEY_DOWN: 
        speed_left, speed_right = -max_speed, -max_speed
    elif key == KEY_RIGHT:  
        speed_left, speed_right = max_speed, -max_speed
    elif key == KEY_LEFT:  
        speed_left, speed_right = -max_speed, max_speed     
    
    return speed_left, speed_right
            
def handle_fencing_action(key: int, arm: Arm):
    # Map keys to their corresponding fencing action functions
    action_map: Dict[int, Callable] = {
        KEY_LUNGE:      fence.lunge,
        KEY_PARRY_HIGH: fence.parry_high,
        KEY_PARRY_LOW:  fence.parry_low,
        KEY_EN_GARDE:   fence.en_garde
    }
    # Execute if key exists in map
    if key in action_map:
        action_map[key]()

def main():
    combot: Combot = Combot()
    timestep = int(combot.getBasicTimeStep())
    arm: Arm = Arm(combot, fc.RIGHT_ARM_CONFIG)
    fence.init(arm) # Initialise fencing module with arm reference
    
    wb.wb_keyboard_enable(timestep)

    # Get references to the wheel motors
    left_wheel = combot.getDevice("wheel_left_joint")
    right_wheel = combot.getDevice("wheel_right_joint")

    # Configure motors for velocity control (set position to infinity)
    left_wheel.setPosition(float('inf'))
    right_wheel.setPosition(float('inf'))
    combot.start_timer()

    counter = 0
    try:
        while combot.step(timestep) != -1:

            key = wb.wb_keyboard_get_key()

            if key > 0:
                max_speed = left_wheel.getMaxVelocity()
                speed_left, speed_right = handle_movement_speed(key, max_speed)

                left_wheel.setVelocity(speed_left)
                right_wheel.setVelocity(speed_right)

                handle_fencing_action(key, arm)
            else:
                left_wheel.setVelocity(0)
                right_wheel.setVelocity(0)

            if fence.check_hit():
                print("Time elapsed: ", combot.get_elapsed_time())
            # combot.move_to_position(Position(3, 1, math.pi), counter)
            # counter +=1
            # move = strat.strategy5(combot)
            # if move is not None:
            #     move()


            if (counter//30)%2==0:
                combot.movement = None
                combot.move_to_position(Position(combot.get_position().x+0.005, 0, 0), counter)
                print(combot.position.x)
            else:
                combot.movement = None
                combot.move_to_position(combot.position, counter)
                
            counter +=1
            # move = strat.strategy7(combot)
            # if move is not None:
            #     move()


            
    except KeyboardInterrupt:   
        print("Controller stopped by user.")
        pass

def train():
    import Qlearning
    import torch

    model_name = "model4"

    print(Qlearning.policy_net)
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
            Qlearning.ACTIONSPACE[action-1]()

            if combot.base_state=="FORWARD":
                if previous == "BACKWARD":
                    combot.movement = None
                    combot.localisation = None
                combot.move_to_position(Position(3.0, 0, 0), counter)
            elif combot.base_state=="BACKWARD":
                if previous == "FORWARD":
                    combot.movement = None
                    combot.localisation = None
                combot.move_to_position(Position(-0.5, 0, 0), counter)
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
            # θ′ ← τ θ + (1 −τ )θ′
            target_net_state_dict = Qlearning.target_net.state_dict()
            policy_net_state_dict = Qlearning.policy_net.state_dict()
            for key in policy_net_state_dict:
                target_net_state_dict[key] = policy_net_state_dict[key]*Qlearning.TAU + target_net_state_dict[key]*(1-Qlearning.TAU)
            Qlearning.target_net.load_state_dict(target_net_state_dict)

            if terminated:
                Qlearning.episode_durations.append(counter)
                # Qlearning.plot_durations()
                break
        
        results.append({'episode': episode, 'length': counter, 'end reason' : endReason , 'reward sum':rewardSum})

    with open("./DQN_training_results/"+model_name+".csv", "w") as f:
        writer = csv.DictWriter(f, fieldnames=['episode', 'length', 'end reason', 'reward sum'])
        writer.writeheader()
        writer.writerows(results)


    
    print('Complete')

    print("Saving NN")
    torch.save(Qlearning.policy_net.state_dict(), "./DQN_states/"+model_name+".pt")

    Qlearning.plot_durations(show_result=True)
    Qlearning.plt.ioff()
    Qlearning.plt.show()

if __name__ == "__main__":
    if ISUSER:
        main()
    elif ISTRAINING:
        train()
