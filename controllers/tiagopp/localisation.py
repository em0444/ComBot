from typing import List

from controller import PositionSensor, DistanceSensor
from controllers.tiagopp.combot import Combot

combot = Combot()

def get_position() -> List[float]:

    position_sensors: List[PositionSensor] = [device for device in combot.devices.values() if isinstance(device, PositionSensor)]
    distance_sensors: List[DistanceSensor] = [device for device in combot.devices.values() if isinstance(device, DistanceSensor)]
    # lidar_sensors = List

    for sensor in distance_sensors:
        print(sensor.name)

    raise NotImplementedError()