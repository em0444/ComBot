from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Tuple


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
        Return if the position is inside the map.
        """

        # If it's outside the main box, return false
        max_x, max_y, min_x, min_y = 5, 2.5, -5, -2.5
        if self.x > max_x or self.x < min_x:
            return False

        if self.y > max_y or self.y < min_y:
            return False

        #(1,1) square in the corner
        if self.x > 4 and self.y>1.5:
            return False

        #(1.5, 1) square in the corner
        if self.x > 3.5 and self.y<-1.5:
            return False

        #(0.5, 2) square in the corner
        if self.x <-4.5 and self.y<-0.5:
            return False

        #(2, 2) square in the corner
        if self.x<-3 and self.y>0.5:
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
