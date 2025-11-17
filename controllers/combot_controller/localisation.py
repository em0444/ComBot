import math
from typing import List

from controller import PositionSensor, DistanceSensor, Lidar
from combot import Combot
import matplotlib.pyplot as plt

combot = Combot()

def get_position() -> List[float]:

    # position_sensors: List[PositionSensor] = [device for device in combot.devices.values() if isinstance(device, PositionSensor)]
    # distance_sensors: List[DistanceSensor] = [device for device in combot.devices.values() if isinstance(device, DistanceSensor)]
    lidar_sensor: Lidar = [device for device in combot.devices.values() if isinstance(device, Lidar)][0]

    lidar_sensor.__init__(name="Velodyne VLP-16", sampling_period=None)
    """
    The Velodyne VLP-16 Sensor:
    - The Tiago robot can be equipped with another lidar sensor, but this one provides a good resolution, and a better FOV. It provides
    - a 360 degree FOV
    - Spread over 3600 points
    
    If you keep this sensor in the default position for tiago++ (i.e. in the base) then it will only be able to measure an FOV for what's within a narrow field of view for it.
    """
    lidar_sensor.enable(int(combot.getBasicTimeStep()))
    range_image : List[float] = lidar_sensor.getRangeImage()

    y = [10 if math.isinf(v) else v for v in getlayer(range_image, 6)]
    plt.plot(y)
    plt.show()
    #
    #
    raise NotImplementedError()


def getlayer(values: List[float], layernum) -> List[float]:
    offset = layernum * 3600
    return values[offset : offset + 3600]



