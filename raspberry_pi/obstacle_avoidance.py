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

    def find_avoid(self, distances, theta_sensors):
        """Compute the final avoidance angle based on vectors."""
        # Replace None values with MAX_OBSTACLE_DISTANCE
        distances = [d if d is not None else self.max_obstacle_distance for d in distances]

        # Calculate weighted sum of distances * theta and sum of distances
        weighted_sum = sum(d * t for d, t in zip(distances, self.theta_sensors))
        total_distance = sum(distances)

        # Compute the avoidance angle alpha
        if total_distance == 0:
            return 0  # Prevent division by zero, default to 0 or another behavior
        alpha = weighted_sum / total_distance

        return alpha
