#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

def wrap_angle(angle): return np.arctan2(np.sin(angle), np.cos(angle))

class FilterConfiguration(object):
    def __init__(self):
        # Process and measurement noise covariance matrices
        self.V = np.diag([0.1, 0.1, 0.05]) ** 2  # Process noise covariance
        # Measurement noise variance (range measurements)
        self.W_range = 0.5 ** 2
        self.W_bearing = (np.pi * 0.5 / 180.0) ** 2

        # Initial conditions for the filter
        self.x0 = np.array([2.0, 3.0, np.pi / 4])
        self.Sigma0 = np.diag([1.0, 1.0, 0.5]) ** 2


class Map(object):
    def __init__(self):
        self.landmarks = np.array([
            [5, 10],
            [15, 5],
            [10, 15]
            # [-20, 20],
            # [-20, -10],
            # [20, -10],
            # [20, 20]
        ])
        self.num_landmarks = 3
        self.landmark_type = "defaults"
    
    def generate_square_grid(self, grid_size=3, spacing=2):
        """Generates a square grid of landmarks centered around the origin."""
        self.landmarks = []
        self.landmark_type = "square_grid"
        
        # Calculate the half-size to center the grid around the origin
        half_size = (grid_size - 1) / 2
        
        # Generate the grid
        for i in range(grid_size):
            for j in range(grid_size):
                x = (i - half_size) * spacing
                y = (j - half_size) * spacing
                self.landmarks.append([x, y])
        
        self.landmarks = np.array(self.landmarks)
        self.num_landmarks = len(self.landmarks)

    def generate_square_boundary(self, side_length=10, num_landmarks_per_side=4):
        """Generates landmarks along the boundary of a square centered at the origin."""
        self.landmarks = []
        self.landmark_type = "square_boundary"

        # Calculate the half-side length to center the square around the origin
        half_side = side_length / 2

        # Number of landmarks total
        self.num_landmarks = num_landmarks_per_side * 4

        # Define the four corners of the square
        corners = [
            (-half_side, half_side),  # Top left
            (half_side, half_side),   # Top right
            (half_side, -half_side),  # Bottom right
            (-half_side, -half_side)  # Bottom left
        ]

        # Distribute landmarks along each side of the square
        for i in range(4):
            start = np.array(corners[i])
            end = np.array(corners[(i + 1) % 4])
            x_values = np.linspace(start[0], end[0], num_landmarks_per_side + 1)[:-1]  # exclude endpoint
            y_values = np.linspace(start[1], end[1], num_landmarks_per_side + 1)[:-1]  # exclude endpoint
            self.landmarks.extend(zip(x_values, y_values))

        self.landmarks = np.array(self.landmarks)
        self.num_landmarks = len(self.landmarks)

    def generate_circular_boundary(self, radius, num_landmarks):
        """Generates landmarks in a circular boundary."""
        self.landmarks = []
        self.landmark_type = "circular_boundary"

        angle_step = 2 * np.pi / num_landmarks
        for i in range(num_landmarks):
            angle = i * angle_step
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            self.landmarks.append([x, y])

        self.landmarks = np.array(self.landmarks)
        self.num_landmarks = num_landmarks

    def generate_circular_grid(self, center=(0, 0), radius=10, spacing=2):
        """Generates landmarks in concentric circles with equal spacing."""
        self.landmarks = []
        self.landmark_type = "circular_grid"

        # Iterate over radial distances from the center, with spacing between circles
        for r in np.arange(0, radius + spacing, spacing):
            # Calculate the number of points for the current circle based on radius
            num_points = int(2 * np.pi * r / spacing) if r != 0 else 1
            angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
            
            # Compute x and y coordinates for each point on the current circle
            for angle in angles:
                x = center[0] + r * np.cos(angle)
                y = center[1] + r * np.sin(angle)
                self.landmarks.append([x, y])
        
        self.landmarks = np.array(self.landmarks)
        self.num_landmarks = len(self.landmarks)
    
    def generate_triangular_boundary(self, num_landmarks, center=(0, 0), side_length=10):
        """Set up landmarks along the boundary of an equilateral triangle."""
        self.landmarks = []
        self.landmark_type = "triangular_boundary"

        # Calculate the vertices of the equilateral triangle
        height = np.sqrt(3) / 2 * side_length
        vertices = [
            (center[0], center[1] + 2 * height / 3),  # Top vertex
            (center[0] - side_length / 2, center[1] - height / 3),  # Bottom left vertex
            (center[0] + side_length / 2, center[1] - height / 3),  # Bottom right vertex
        ]
        
        # Distribute points along each side
        num_points_per_side = num_landmarks // 3
        remainder = num_landmarks % 3
        
        for i in range(3):
            start, end = vertices[i], vertices[(i + 1) % 3]
            
            # Calculate points for this side
            side_points = num_points_per_side + (1 if i < remainder else 0)
            x_values = np.linspace(start[0], end[0], side_points + 1)[:-1]  # exclude endpoint
            y_values = np.linspace(start[1], end[1], side_points + 1)[:-1]  # exclude endpoint
            
            # Append these points to landmarks
            for x, y in zip(x_values, y_values):
                self.landmarks.append([x, y])

        # Convert to array and update class variables
        self.landmarks = np.array(self.landmarks)
        self.num_landmarks = len(self.landmarks)
    
    def generate_triangular_grid(self, center=(0, 0), spacing=2, num_rows=5):
        """Set up landmarks in a triangular (hexagonal) grid pattern."""
        self.landmarks = []
        self.landmark_type = "triangular_grid"

        # Calculate the height of an equilateral triangle with the given spacing
        row_height = np.sqrt(3) / 2 * spacing

        for row in range(num_rows):
            # Determine the horizontal offset for this row to achieve the triangular pattern
            y = center[1] + row * row_height
            num_cols = num_rows - row  # Decrease the number of points per row as you go up
            x_start = center[0] - (num_cols - 1) * spacing / 2
            
            for col in range(num_cols):
                x = x_start + col * spacing
                self.landmarks.append([x, y])

        self.landmarks = np.array(self.landmarks)
        self.num_landmarks = len(self.landmarks)


class RobotEstimator(object):

    def __init__(self, filter_config, map):
        # Variables which will be used
        self._config = filter_config
        self._map = map

    # This nethod MUST be called to start the filter
    def start(self):
        self._t = 0
        self._set_estimate_to_initial_conditions()

    def set_control_input(self, u):
        self._u = u

    # Predict to the time. The time is fed in to
    # allow for variable prediction intervals.
    def predict_to(self, time):
        # What is the time interval length?
        dt = time - self._t

        # Store the current time
        self._t = time

        # Now predict over a duration dT
        self._predict_over_dt(dt)

    # Return the estimate and its covariance
    def estimate(self):
        return self._x_est, self._Sigma_est

    # This method gets called if there are no observations
    def copy_prediction_to_estimate(self):
        self._x_est = self._x_pred
        self._Sigma_est = self._Sigma_pred

    # This method sets the filter to the initial state
    def _set_estimate_to_initial_conditions(self):
        # Initial estimated state and covariance
        self._x_est = self._config.x0
        self._Sigma_est = self._config.Sigma0

    # Predict to the time
    def _predict_over_dt(self, dt):
        v_c = self._u[0]
        omega_c = self._u[1]
        V = self._config.V

        # Predict the new state
        self._x_pred = self._x_est + np.array([
            v_c * np.cos(self._x_est[2]) * dt,
            v_c * np.sin(self._x_est[2]) * dt,
            omega_c * dt
        ])
        self._x_pred[-1] = np.arctan2(np.sin(self._x_pred[-1]),
                                      np.cos(self._x_pred[-1]))

        # Predict the covariance
        A = np.array([
            [1, 0, -v_c * np.sin(self._x_est[2]) * dt],
            [0, 1,  v_c * np.cos(self._x_est[2]) * dt],
            [0, 0, 1]
        ])

        self._kf_predict_covariance(A, self._config.V * dt)

    # Predict the EKF covariance; note the mean is
    # totally model specific, so there's nothing we can
    # clearly separate out.
    def _kf_predict_covariance(self, A, V):
        self._Sigma_pred = A @ self._Sigma_est @ A.T + V

    # Implement the Kalman filter update step.
    def _do_kf_update(self, nu, C, W):

        # Kalman Gain
        SigmaXZ = self._Sigma_pred @ C.T
        SigmaZZ = C @ SigmaXZ + W
        K = SigmaXZ @ np.linalg.inv(SigmaZZ)

        # nu = nu.flatten()
        # State update
        self._x_est = self._x_pred + K @ nu

        # Covariance update
        self._Sigma_est = (np.eye(len(self._x_est)) - K @ C) @ self._Sigma_pred

    def update_from_landmark_range_observations(self, y_range):

        # Predicted the landmark measurements and build up the observation Jacobian
        y_pred = []
        C = []
        x_pred = self._x_pred
        for lm in self._map.landmarks:

            dx_pred = lm[0] - x_pred[0]
            dy_pred = lm[1] - x_pred[1]
            range_pred = np.sqrt(dx_pred**2 + dy_pred**2)
            y_pred.append(range_pred)

            # Jacobian of the measurement model
            C_range = np.array([
                -(dx_pred) / range_pred,
                -(dy_pred) / range_pred,
                0
            ])
            C.append(C_range)
        # Convert lists to arrays
        C = np.array(C)
        y_pred = np.array(y_pred)

        # Innovation. Look new information! (geddit?)
        nu = y_range - y_pred

        # Since we are oberving a bunch of landmarks
        # build the covariance matrix. Note you could
        # swap this to just calling the ekf update call
        # multiple times, once for each observation,
        # as well
        W_landmarks = self._config.W_range * np.eye(len(self._map.landmarks))
        self._do_kf_update(nu, C, W_landmarks)

        # Angle wrap afterwards
        self._x_est[-1] = np.arctan2(np.sin(self._x_est[-1]),
                                     np.cos(self._x_est[-1]))
        
    def update_from_landmark_range_bearing_observations(self, y_range_bearing):

        # Predicted the landmark measurements and build up the observation Jacobian
        y_pred = []
        C = []
        x_pred = self._x_pred
        for lm in self._map.landmarks:

            dx_pred = lm[0] - x_pred[0]
            dy_pred = lm[1] - x_pred[1]
            range_pred = np.sqrt(dx_pred**2 + dy_pred**2)
            bearing_pred = wrap_angle(np.arctan2(dy_pred, dx_pred) - x_pred[2])
            
            y_pred.append([range_pred, bearing_pred])

            # Jacobian of the measurement model
            C_range = np.array([
                [-(dx_pred) / range_pred, -(dy_pred) / range_pred, 0],
                [(dy_pred) / (range_pred**2), -(dx_pred) / (range_pred**2), -1]
            ])
            C.append(C_range)
        # Convert lists to arrays
        # C = np.array(C)
        C = np.vstack(C)
        y_pred = np.array(y_pred)
        y_pred = np.array(y_pred).flatten()
        y_range_bearing = y_range_bearing.flatten()

        # Innovation. Look new information! (geddit?)
        nu = y_range_bearing - y_pred
        for i in range(1, len(nu), 2):
            nu[i] = wrap_angle(nu[i])

        # Since we are oberving a bunch of landmarks
        # build the covariance matrix. Note you could
        # swap this to just calling the ekf update call
        # multiple times, once for each observation,
        # as well
        # W_landmarks = self._config.W_range * np.eye(len(self._map.landmarks))
        # self._do_kf_update(nu, C, W_landmarks)

        # print(f"W_range: {self._config.W_range} \n")
        # print(f"W_bearing: {self._config.W_bearing} \n")
        
        num_landmarks = len(self._map.landmarks)
        # W_landmarks = np.zeros((2 * num_landmarks, 2 * num_landmarks))
        # # print(W_landmarks)
        # for i in range(num_landmarks):
        #     W_landmarks[2 * i, 2 * i] = self._config.W_range
        #     W_landmarks[2 * i + 1, 2 * i + 1] = self._config.W_bearing
        # print(W_landmarks)

        W_landmarks = np.kron(np.eye(num_landmarks), np.diag([self._config.W_range, self._config.W_bearing]))
        # print(W_landmarks)

        self._do_kf_update(nu, C, W_landmarks)

        # Angle wrap afterwards
        self._x_est[-1] = np.arctan2(np.sin(self._x_est[-1]),
                                     np.cos(self._x_est[-1]))

