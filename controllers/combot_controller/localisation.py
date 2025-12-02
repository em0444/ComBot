from __future__ import annotations
import math
import random
from copy import copy
from dataclasses import dataclass

from controllers.combot_controller.shared_dataclasses import Position
from typing import List, Tuple

from controller import PositionSensor, Lidar
from controllers.combot_controller.combot import Combot

global combot

# distance_sensors: List[DistanceSensor] = [device for device in combot.devices.values() if isinstance(device, DistanceSensor)]

class InertialHeading:
    def __init__(self):
        self.inertial_unit = combot.getDevice("inertial unit")
        self.inertial_unit.__init__("inertial unit")

    def get_heading_in_radians(self) -> float:
        roll_pitch_yaw = self.inertial_unit.getRollPitchYaw()
        heading_in_radians = roll_pitch_yaw[2]
        if heading_in_radians < 0:
            heading_in_radians += 2 * math.pi
        # print(f"heading in radians: {heading_in_radians}")
        return heading_in_radians

class Localisation:
    def __init__(self, combot_obj: Combot, num_particles=50):
        global combot
        combot = combot_obj

        self.lidar_array = LidarArray()
        self.wheel_odometry = WheelOdometry()
        self.inertial_heading = InertialHeading()

        initial_random_particles = [Particle(Position(0, 0, 2 * math.pi)) for _ in range(num_particles)]
        self.particles = initial_random_particles
        self.num_particles = num_particles

    def update_internal_position_model(self) -> Position:

        odometry_change_data = self.wheel_odometry.get_odometry_change_since_last_query()
        lidar_data = self.lidar_array.get_lidar_map()

        for particle in self.particles:
            particle.calculate_log_likelihood(odometry_change_data, lidar_data)

        max_log_likelihood, min_log_likelihood = max([p.log_likelihood for p in self.particles]), min([p.log_likelihood for p in self.particles])

        for particle in self.particles:
            particle.calculate_normalised_weight(max_log_likelihood, min_log_likelihood, self.num_particles)

        best_particle = sorted(self.particles, key=lambda p: p.normalised_weight, reverse=True)[0]

        for particle in self.particles:
            particle.position = copy(best_particle.position)
            particle.position = Position(x=particle.position.x, y=particle.position.y, heading_in_radians=self.inertial_heading.get_heading_in_radians())

        return copy(best_particle.position)



        # test_particles = []
        # for i in range(5):
        #     for j in range(5):
        #         some_particle = Particle(Position(x=(i*0.5), y=(j*0.5), heading_in_radians=math.pi))
        #         test_particles.append(some_particle)
        #
        # self.particles = test_particles
        #
        # for particle in self.particles:
        #     particle.calculate_log_likelihood(odometry_change_data, lidar_data)
        #
        # max_log_likelihood, min_log_likelihood = max([p.log_likelihood for p in self.particles]), min([p.log_likelihood for p in self.particles])
        #
        # for particle in self.particles:
        #     particle.calculate_normalised_weight(max_log_likelihood, min_log_likelihood, self.num_particles)
        #
        # self.particles = random.choices(self.particles, weights=[p.normalised_weight for p in self.particles], k=self.num_particles) # Perform the actual resampling step
        #
        # best_three_particles = sorted(self.particles, key=lambda p: p.normalised_weight, reverse=True)[:3]
        # print((best_three_particles[0].position.x + best_three_particles[1].position.x + best_three_particles[2].position.x)/3, (best_three_particles[0].position.y + best_three_particles[1].position.y + best_three_particles[2].position.y)/3 )
        # return((best_three_particles[0].position.x + best_three_particles[1].position.x + best_three_particles[2].position.x)/3, (best_three_particles[0].position.y + best_three_particles[1].position.y + best_three_particles[2].position.y)/3 )

    def get_enemy_position(self):
        combot.update_internal_position_model()
        raise NotImplementedError()


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
        self.lidar_sensor.__init__("Velodyne VLP-16", int(combot.getBasicTimeStep()))
        self.lidar_sensor.enable(int(combot.getBasicTimeStep()))

    def plot(self):
        """
        Plot the received lidar data for debugging purposes.
        """
        import matplotlib.pyplot as plt #Lazy import to stop compiler complaining
        y = [10 if math.isinf(v) else v for v in self.lidar_sensor.getRangeImageArray()[6]]
        plt.plot(y)
        plt.show()

    def get_lidar_map(self, num_distances=50) -> List[LidarRay]: # TODO this needs to make proper use of heading data !!
        range_image = self.lidar_sensor.getRangeImageArray()[6]
        step = len(range_image)//num_distances
        range_image = range_image[::step]
        mid = len(range_image) // 2
        left_vision = [range_image[i] for i in range(mid)]
        right_vision = [range_image[i] for i in range(mid, len(range_image))]

        indexed_left_vision = [((3*math.pi)/2 + (i/len(left_vision))*(math.pi/2), v) for i, v in enumerate(left_vision)]
        indexed_right_vision = [(i/len(left_vision)*math.pi/2, v)  for i, v in enumerate(right_vision)]
        vision = [(i, v) for (i, v) in indexed_left_vision + indexed_right_vision if not (math.isinf(v) or v < 0.2)]
        vision = [LidarRay(distance=distance, angle_in_radians=angle) for angle, distance in vision]

        return vision


class Particle:
    def __init__(self, position=None):
        if position is None:
            self.position: Position = Position.make_random()
        else:
            self.position: Position = position
        self.log_likelihood = 0

    def calculate_log_likelihood(self, odometry_change_data: OdometryChange, real_lidar_data: List[LidarRay]) -> None:

        if not self.position.is_in_map(): # Particle somehow invalid
            self.position = Position(0, 0, math.pi)

        real_lidar_data = real_lidar_data.copy()
        self.add_odometry_to_current_position_with_uncertainty(odometry_change_data)
        expected_lidar_data = self.expected_lidar_data([ray.angle_in_radians for ray in real_lidar_data]) #Generate expected lidar data. Feed it the angles for which we want expected values.

        sensor_standard_deviation = 5

        error_summation = 0
        while not expected_lidar_data == []:
            expected: LidarRay = expected_lidar_data.pop()
            real: LidarRay = real_lidar_data.pop()
            error = math.pow((expected.distance - real.distance), 2) # MSE Error
            error_summation += error

        log_likelihood = - error_summation / (2 * math.pow(sensor_standard_deviation, 2))
        self.log_likelihood = log_likelihood

    def expected_lidar_data(self, angles_to_generate_rays_for_in_radians: List[float]) -> List[LidarRay]:
        """
        For the particle's position, generate the data that the LidarArray is most likely to produce.
        """
        ray_data: List[LidarRay] = []
        for angle in angles_to_generate_rays_for_in_radians: # Use ray angles that the Lidar is actually producing to allow for better likelihood prediction.
            ray = LidarRay(distance=0, angle_in_radians=angle)
            while self._end_of_cast_ray_is_in_map(ray):
               ray = ray.change_distance(0.05) # If it's still in the map, make the ray slightly longer. Keep checking until it just touches a wall.
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
        heading = self.position.heading_in_radians

        delta_s = (delta_sl + delta_sr) / 2 # Update x and y according to equations
        delta_x = delta_s * math.cos(heading + delta_theta / 2)
        delta_y = delta_s * math.sin(heading + delta_theta / 2)

        delta_x += random.gauss(0, 0.005) # Add gaussian uncertainty
        delta_y += random.gauss(0, 0.005)
        delta_theta = (delta_theta + random.gauss(0, 0.005)) % (2 * math.pi)

        self.position = self.position.add(delta_x, delta_y, delta_theta) # Update the position

    def calculate_normalised_weight(self, max_log_likelihood, min_log_likelihood, num_particles) -> None:
        if max_log_likelihood == min_log_likelihood:
            self.normalised_weight = 1 / num_particles
        else:
            self.normalised_weight = (self.log_likelihood - min_log_likelihood) / (max_log_likelihood - min_log_likelihood)


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
