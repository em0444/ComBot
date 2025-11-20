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


        # initial_random_particles = [Particle.generate_random_position() for _ in range(num_particles)]
        #
        # self.particles = initial_random_particles
        # self.i = 0


    def get_position(self) -> Tuple[float, float]:

        # self.wheel_odometry.add_odometry_with_uncertainty(self.particles)
        # [particle.generate_ray_march_expected_sensor_data() for particle in self.particles]
        self.lidar_array.get_angle_distances()

        return (0,0)

        # raise NotImplementedError()

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

    def add_odometry_with_uncertainty(self, particles):
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
    # Wrapper for the Hokuyo URG-04LX-UG01 sensor
    def __init__(self):
        self.lidar_sensor: Lidar = [device for device in combot.devices.values() if isinstance(device, Lidar)][0]
        print(self.lidar_sensor.name)
        self.lidar_sensor.__init__("Hokuyo URG-04LX-UG01", int(combot.getBasicTimeStep()))
        self.lidar_sensor.enable(int(combot.getBasicTimeStep()))

    def plot(self):
        y = [10 if math.isinf(v) else v for v in self.lidar_sensor.getRangeImage()]
        plt.plot(y)
        plt.show()

    def get_angle_distances(self, num_distances=20) -> List[Tuple[float, float]]: #List of (angle in radians, distance)
        range_image = self.lidar_sensor.getRangeImage()
        print(self.lidar_sensor.getFov())
        mid = len(range_image) // 2
        left_vision = [range_image[i] for i in range(mid)]
        right_vision = [range_image[i] for i in range(mid, len(range_image))]

        indexed_left_vision = [((3*math.pi)/2 + (i/len(left_vision))*(math.pi/2), v) for i, v in enumerate(left_vision)]
        indexed_right_vision = [(i/len(left_vision)*math.pi/2, v)  for i, v in enumerate(right_vision)]
        vision = [(i, v) for (i, v) in indexed_left_vision + indexed_right_vision if not math.isinf(v)]

        return random.sample(vision, num_distances)


class Particle:
    max_x, max_y, min_x, min_y = 5, 5, -5, -5
    def __init__(self, x: float, y: float):
        self.x = max(min(x, self.max_x), self.min_x)
        self.y = max(min(y, self.max_y), self.min_y)
        self.heading = 0

    def is_possible_position(self, x: float, y: float) -> bool:
        return (self.min_x < x < self.max_x) and (self.min_y < y < self.max_y)

    def generate_ray_march_expected_sensor_data(self, angles: List[float]) -> List[Tuple[float, float]]:
        expected_sensor_data: List[Tuple[float, float, bool]] = [(angle, 0.0, False) for angle in angles] # List of (angle, distance, finished)
        all_done = False
        while not all_done:
            all_done = True
            for i, (angle, distance, done) in enumerate(expected_sensor_data):
                if done:
                    continue
                all_done = False
                distance+=1
                if not self.is_in_map(angle, distance):
                    done = True
                expected_sensor_data[i] = (angle, distance, done)
            if all_done:
                break
        return [(angle, distance) for (angle, distance, done) in expected_sensor_data]

    def is_in_map(self, angle, distance) -> bool:
        position_x = self.x + math.cos(angle) * distance
        position_y = self.y + math.sin(angle) * distance
        return (self.min_x < position_x < self.max_x) and (self.min_y < position_y < self.max_y)

    def calculate_weight(self, lidar_array: LidarArray):
        real_sensor_values = lidar_array.get_angle_distances()
        expected_sensor_means = self.generate_ray_march_expected_sensor_data([angle[0] for angle in real_sensor_values])
        sensor_std = 0.5

        for value in zip(expected_sensor_means, real_sensor_values):
            print("here!")
            #find the probability of that value, add it together to get the log likelihood

    @staticmethod
    def generate_random_position():
        return Particle(random.uniform(Particle.min_x, Particle.max_x), random.uniform(Particle.min_y, Particle.max_y))
