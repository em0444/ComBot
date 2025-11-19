import math
import random
from time import sleep
from typing import List, Tuple

from controller import PositionSensor, Lidar
import matplotlib.pyplot as plt

from controllers.combot_controller.combot import Combot

global combot

class Localisation:
    def __init__(self, combot_obj: Combot, num_particles=5):
        global combot
        combot = combot_obj
        # distance_sensors: List[DistanceSensor] = [device for device in combot.devices.values() if isinstance(device, DistanceSensor)]
        self.num_particles = num_particles

        self.lidar_array = LidarArray()
        self.wheel_odometry = WheelOdometry()

        initial_random_particles = [Position.generate_random_position() for _ in range(num_particles)]

        self.particles = initial_random_particles
        self.i = 0


    def get_position(self) -> Tuple[float, float]:

        self.wheel_odometry.add_odometry_with_uncertainty(self.particles)

        return (0,0)

        # raise NotImplementedError()

class Position:
    max_x, max_y, min_x, min_y = 5, 5, -5, -5
    def __init__(self, x: float, y: float):
        self.x = max(min(x, self.max_x), self.min_x)
        self.y = max(min(y, self.max_y), self.min_y)

    def is_possible_position(self, x: float, y: float) -> bool:
        return (self.min_x < x < self.max_x) and (self.min_y < y < self.max_y)

    @staticmethod
    def generate_random_position():
        return Position(random.uniform(Position.min_x, Position.max_x), random.uniform(Position.min_y, Position.max_y))

class WheelOdometry:
    # Moving striaght into the wall gives us values 43, 43, when we reach it. Dividing by actual distance travelled gets us a (rounded) scale factor of 10.
    def __init__(self):
        self.left_sensor: PositionSensor = [device for device in combot.devices.values() if device.name == "wheel_left_joint_sensor"][0]
        self.right_sensor: PositionSensor = [device for device in combot.devices.values() if device.name == "wheel_right_joint_sensor"][0]
        self.left_sensor.enable(int(combot.getBasicTimeStep()))
        self.right_sensor.enable(int(combot.getBasicTimeStep()))

        self.stored_odometry: Tuple[float, float] = self.get_current_odometry()
        self.stored_heading: float = 0.0
        self.axle_radius: float = 0.4 # TODO double check this value
        self.wheel_scale_factor = 0.1

    def get_current_odometry(self) -> Tuple[float, float]:
        return self.left_sensor.value, self.right_sensor.value

    def get_odometry_change_since_last_query(self) -> Tuple[float, float]:
        last_left, last_right = self.stored_odometry
        current_left, current_right = self.get_current_odometry()
        self.stored_odometry = (current_left, current_right)
        return (current_left - last_left) * self.wheel_scale_factor, (current_right - last_right) * self.wheel_scale_factor

    def add_odometry_with_uncertainty(self, particles: List[Position]):
        # Apply equations from week 2 to get robot's change in x / y
        (delta_sl, delta_sr) = self.get_odometry_change_since_last_query()
        delta_theta = (delta_sl - delta_sr) / self.axle_radius
        self.stored_heading += delta_theta
        delta_s = (delta_sl + delta_sr) / 2
        delta_x = delta_s * math.cos(self.stored_heading + delta_theta / 2)
        delta_y = delta_s * math.sin(self.stored_heading + delta_theta / 2)

        for particle in particles:
            particle.x += delta_x + random.gauss(0, 0.005)
            particle.y += delta_y + random.gauss(0, 0.005)


class LidarArray:
    """
    Wrapper for The Velodyne VLP-16 Sensor:
    - The Tiago robot can be equipped with another lidar sensor, but this one provides a good resolution, and a better FOV. It provides
    - a 360 degree FOV
    - Spread over 3600 points

    If you keep this sensor in the default position for tiago++ (i.e. in the base) then it will only be able to measure an FOV for what's within a narrow field of view for it.
    """
    def __init__(self):
        self.lidar_sensor: Lidar = [device for device in combot.devices.values() if isinstance(device, Lidar)][0]
        print(self.lidar_sensor.name)
        self.lidar_sensor.__init__("")
        sleep(1)
        self.lidar_sensor.enable(int(combot.getBasicTimeStep()))
        sleep(1)
        self.range_image: List[float] = self.lidar_sensor.getRangeImage()

    def plot(self):
        for i in range(16):
            y = [10 if math.isinf(v) else v for v in self.get_layer(layernum=i)]
            plt.title(f"Layer {i}")
            plt.plot(y)
            plt.show()

    def get_layer(self, layernum) -> List[float]:
        offset = layernum * 3600
        return self.range_image[offset:offset + 3600]

    def get_best_layer(self) -> List[float]:
        return self.get_layer(6) # Layer 6 will show the walls/opponents nicely... Might make the logic here more intelligent, later.

    def get_2d_mapped_sensor_data(self) -> List[Tuple[float, float]]:
        return [((i*2*math.pi)/3600, self.get_best_layer()[i]) for i in range(3600)]
