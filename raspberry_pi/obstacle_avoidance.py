import math
from constants import MAX_OBSTACLE_DISTANCE, THETA_SENSORS

class ObstacleAvoidance:
    def __init__(self):
        self.max_obstacle_distance = MAX_OBSTACLE_DISTANCE
        self.theta_sensors = THETA_SENSORS

    def obstacle_pos(self, distance, theta_sensor, X_R, Y_R, Phi_R):
        """Calculate the obstacle position."""
        x_obs = X_R + distance * math.cos(theta_sensor * math.pi / 180 + Phi_R)
        y_obs = Y_R + distance * math.sin(theta_sensor * math.pi / 180 + Phi_R)
        return x_obs, y_obs

    def find_avoid(self, distances, theta_sensors=None):
        """Bubble rebound angle: the distance-weighted average of the sensor angles."""
        if theta_sensors is None:
            theta_sensors = self.theta_sensors
        # Anything farther than the range (or not seen at all) counts as free space
        distances = [
            self.max_obstacle_distance if d is None else min(d, self.max_obstacle_distance)
            for d in distances
        ]

        weighted_sum = sum(d * t for d, t in zip(distances, theta_sensors))
        total_distance = sum(distances)

        if total_distance == 0:
            return 90  # boxed in on every side: keep the current heading
        return weighted_sum / total_distance
