from __future__ import annotations
import math
import random
from dataclasses import dataclass
from typing import List, Tuple, Optional

from controller import PositionSensor, Lidar
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

        initial_random_particles = [Particle() for _ in range(num_particles)]
        self.particles = initial_random_particles

    def get_position(self) -> Tuple[float, float]:

        odometry_change_data = self.wheel_odometry.get_odometry_change_since_last_query()
        lidar_data = self.lidar_array.get_lidar_map()

        weights = [particle.calculate_weight(odometry_change_data, lidar_data) for particle in self.particles]
        print(weights)

        return 0,0


class WheelOdometry:
    # Moving striaght into the wall gives us values 43, 43, when we reach it. Dividing by actual distance travelled gets us a (rounded) scale factor of 10.
    def __init__(self):
        self.left_sensor: PositionSensor = [device for device in combot.devices.values() if device.name == "wheel_left_joint_sensor"][0] # Find the sensors for the combot, and enable them.
        self.right_sensor: PositionSensor = [device for device in combot.devices.values() if device.name == "wheel_right_joint_sensor"][0]
        self.left_sensor.enable(int(combot.getBasicTimeStep()))
        self.right_sensor.enable(int(combot.getBasicTimeStep()))

        self.stored_odometry: Tuple[float, float] = self.get_current_odometry()
        self.wheel_scale_factor = 0.1

    def get_current_odometry(self) -> Tuple[float, float]:
        return self.left_sensor.value, self.right_sensor.value

    def get_odometry_change_since_last_query(self) -> OdometryChange:
        last_left, last_right = self.stored_odometry
        current_left, current_right = self.get_current_odometry()
        self.stored_odometry = (current_left, current_right)
        change_in_left = (current_left - last_left) * self.wheel_scale_factor
        change_in_right = (current_right - last_right) * self.wheel_scale_factor
        return OdometryChange(change_in_left, change_in_right)


class LidarArray:
    # Wrapper for the Hokuyo URG-04LX-UG01 sensor
    def __init__(self):
        self.lidar_sensor: Lidar = [device for device in combot.devices.values() if isinstance(device, Lidar)][0]
        self.lidar_sensor.__init__("Hokuyo URG-04LX-UG01", int(combot.getBasicTimeStep()))
        self.lidar_sensor.enable(int(combot.getBasicTimeStep()))

    def plot(self):
        """
        Plot the received lidar data for debugging purposes.
        """
        import matplotlib.pyplot as plt #Lazy import to stop compiler complaining
        y = [10 if math.isinf(v) else v for v in self.lidar_sensor.getRangeImage()]
        plt.plot(y)
        plt.show()

    def get_lidar_map(self, num_distances=20) -> List[LidarRay]: # TODO this needs to make proper use of heading data !!
        range_image = self.lidar_sensor.getRangeImage()
        mid = len(range_image) // 2
        left_vision = [range_image[i] for i in range(mid)]
        right_vision = [range_image[i] for i in range(mid, len(range_image))]

        indexed_left_vision = [((3*math.pi)/2 + (i/len(left_vision))*(math.pi/2), v) for i, v in enumerate(left_vision)]
        indexed_right_vision = [(i/len(left_vision)*math.pi/2, v)  for i, v in enumerate(right_vision)]
        vision = [(i, v) for (i, v) in indexed_left_vision + indexed_right_vision if not math.isinf(v)]
        vision = [LidarRay(distance=distance, angle_in_radians=angle) for distance, angle in vision]

        return random.sample(vision, num_distances)


class Particle:
    def __init__(self, position=None):
        if position is None:
            self.position: Position = Position.make_random()
        else:
            self.position: Position = position

    def calculate_weight(self, odometry_change_data: OdometryChange, real_lidar_data: List[LidarRay]) -> float:
        real_lidar_data = real_lidar_data.copy()
        self.add_odometry_to_current_position_with_uncertainty(odometry_change_data)
        expected_lidar_data = self.expected_lidar_data([ray.angle_in_radians for ray in real_lidar_data]) #Generate expected lidar data. Feed it the angles for which we want expected values.

        sensor_standard_deviation = 0.5

        error_summation = 0
        while not expected_lidar_data == []:
            expected: LidarRay = expected_lidar_data.pop()
            real: LidarRay = real_lidar_data.pop()
            error = math.pow((expected.distance - real.distance), 2)
            error_summation += error

        log_likelihood = -(1/2*math.pow(sensor_standard_deviation, 2)) * error_summation
        return log_likelihood

    def expected_lidar_data(self, angles_to_generate_rays_for_in_radians: List[float]) -> List[LidarRay]:
        """
        For the particle's position, generate the data that the LidarArray is most likely to produce.
        """
        ray_data: List[LidarRay] = []
        for angle in angles_to_generate_rays_for_in_radians: # Use ray angles that the Lidar is actually producing to allow for better likelihood prediction.
            ray = LidarRay(distance=0, angle_in_radians=angle)
            while not self._end_of_cast_ray_is_in_map(ray):
               ray = ray.change_distance(0.05) # If it's not in the map, make the ray slightly longer. Keep checking until it just touches a wall.
            ray_data.append(ray)
        return ray_data

    def _end_of_cast_ray_is_in_map(self, ray: LidarRay) -> bool:
        """
        Adds on a LidarRay to the particle's position, and returns if it's in the map.
        """
        objective_ray_direction: float = (self.position.heading_in_radians + ray.angle_in_radians) % (2 * math.pi) # Find what direction the ray is coming from

        ray_end_x = self.position.x + math.cos(objective_ray_direction) * ray.distance # Add to current position and convert to cartesian coords.
        ray_end_y = self.position.y + math.sin(objective_ray_direction) * ray.distance

        return Position(ray_end_x, ray_end_y, self.position.heading_in_radians).is_in_map()

    @staticmethod
    def make_new_particle() -> Particle:
        return Particle(position=Position.make_random())

    def add_odometry_to_current_position_with_uncertainty(self, odometry_change_data: OdometryChange):
        """
        Apply Week 2 Odometry Equations to update particle position, adding uncertainty.
        """
        delta_sl, delta_sr, axle_radius = odometry_change_data.as_tuple()

        delta_theta = (delta_sl - delta_sr) / axle_radius # Calculate the new heading based on axle radius
        new_heading = (self.position.heading_in_radians + delta_theta ) % (2 * math.pi)

        delta_s = (delta_sl + delta_sr) / 2 # Update x and y according to equations
        delta_x = delta_s * math.cos(new_heading + delta_theta / 2)
        delta_y = delta_s * math.sin(new_heading + delta_theta / 2)

        delta_x += random.gauss(0, 0.005) # Add gaussian uncertainty
        delta_y += random.gauss(0, 0.005)
        delta_theta = (delta_theta + random.gauss(0, 0.005)) % (2 * math.pi)

        self.position = self.position.add(delta_x, delta_y, delta_theta) # Update the position

@dataclass(frozen=True)
class OdometryChange:
    delta_left_wheel: float
    delta_right_wheel: float
    axle_radius: float = 0.4

    def as_tuple(self) -> Tuple[float, float, float]:
        return self.delta_left_wheel, self.delta_right_wheel, self.axle_radius

@dataclass(frozen=True)
class LidarRay:
    distance: float
    angle_in_radians: float

    def as_tuple(self) -> Tuple[float, float]:
        return self.distance, self.angle_in_radians

    def change_distance(self, distance_to_add):
        return LidarRay(self.distance + distance_to_add, self.angle_in_radians)

@dataclass(frozen=True)
class Position:
    x: float
    y: float
    heading_in_radians: float

    def as_tuple(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.heading_in_radians

    def add(self, delta_x, delta_y, delta_theta) -> Position:
        return Position(self.x + delta_x, self.y + delta_y, (self.heading_in_radians + delta_theta) % (2 * math.pi))

    def is_in_map(self) -> bool:
        """
        Return if the position is inside the map. Right now this function is simple, given the room is square, but it could be more complicated for a more complicated room shape/obstacles
        """
        max_x, max_y, min_x, min_y = 5, 5, -5, -5
        if self.x > max_x or self.x < min_x:
            return False
        if self.y > max_y or self.y < min_y:
            return False
        return True

    @classmethod
    def make_random(cls) -> Position:
        """
        Make a new random position within the map boundary. If it's not legal (i.e. obstructed by an obstacle) try again.
        """
        candidate_position = Position(random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(0, (2 * math.pi)))
        if candidate_position.is_in_map():
            return candidate_position
        else:
            return Position.make_random()
