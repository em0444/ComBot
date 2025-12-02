"""combot controller."""

from controller import wb as c_webots_api, \
    Motor  # The wb package gives you all the C-like methods, but the controller package wraps most of them in nicer-to-use classes.
from training_combot import TrainerCombot
import strategy as strat
decideMove = lambda : None
wb = c_webots_api.wb

trainer: TrainerCombot = TrainerCombot()

timestep = int(trainer.getBasicTimeStep())

# Main loop:
while trainer.step(timestep) != -1:

    move = strat.strategy10(trainer)
    if move is not None:
        move()
        print(trainer.get_enemy_position(),trainer.get_position())
    pass

# Enter here exit cleanup code.
