from typing import List

from controllers.tiagopp.combot import Combot

combot = Combot()

def get_position() -> List[float]:
    for device in combot.devices.values():
        print(type(device))
    raise NotImplementedError()