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

            if (counter//30)%2==0:
                combot.movement = None
                combot.move_to_position(Position(combot.get_position().x+0.005, 0, 0), counter)
                print(combot.position.x)
            else:
                combot.movement = None
                combot.move_to_position(combot.position, counter)
                print("aaaa")
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
    print(Qlearning.policy_net)
    combot: Combot = Combot()
    combotNode = combot.getSelf()
    combotEnemy = combot.getFromDef("OPP")
    timestep = int(combot.getBasicTimeStep())
    arm: Arm = Arm(combot, fc.RIGHT_ARM_CONFIG)
    fence.init(arm) # Initialise fencing module with arm reference
    

    combotNode.saveState("Init1")
    combotEnemy.saveState("Init2")
    print(combotNode.getOrientation(),'\n----------------------')
    for episode in range(0,Qlearning.num_episodes):
        combotNode.loadState("Init1")
        combotEnemy.loadState("Init2")
        combot.reset()
        state = Qlearning.getState(combot)
        stepSuccess = 0
        counter = 0
        print("reset")
        from fencing_actions import check_hit
        while stepSuccess != -1:
            action = Qlearning.select_action(state)
            Qlearning.ACTIONSPACE[action-1]()

            if combot.base_state=="FORWARD":
                combot.movement = None
                combot.move_to_position(Position(combot.get_position().x+0.05, 0, 0), counter)
            elif combot.base_state=="BACKWARD":
                combot.movement = None
                combot.move_to_position(Position(combot.get_position().x-0.05, 0, 0), counter)
            else:
                combot.movement = None
                combot.move_to_position(Position(combot.get_position().x, 0, 0), counter)
            counter+=1

            stepSuccess = combot.step(timestep)

            observation = Qlearning.getState(combot)
            if counter < 2500:#End episode if it takes longer than a minute
                reward, terminated = Qlearning.getReward(combotNode,combotEnemy)
            else:
                reward = (torch.tensor([-1000], device=Qlearning.device),True)
                terminated = True


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
                Qlearning.plot_durations()
                break


    
    print('Complete')

    print("Saving NN")
    torch.save(Qlearning.policy_net.state_dict(), "./DQN_states/first_model.pt")

    Qlearning.plot_durations(show_result=True)
    Qlearning.plt.ioff()
    Qlearning.plt.show()

if __name__ == "__main__":
    if ISUSER:
        main()
    elif ISTRAINING:
        train()
