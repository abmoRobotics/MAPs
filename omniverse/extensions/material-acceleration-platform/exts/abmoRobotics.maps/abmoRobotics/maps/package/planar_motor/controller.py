from ..base import OmniverseInterface
import numpy as np


class Controller:
    """PD controller for a single joint"""
    def __init__(self, p_gain: np.ndarray, d_gain: np.ndarray, max_veloicty: np.ndarray):
        self.p_gain = p_gain
        self.d_gain = d_gain
        self.max_veloicty = max_veloicty
        # self.omni_interface = OmniverseInterface()

    def get_control_signal(self, current_position: np.ndarray, target_position: np.ndarray, current_velocity: np.ndarray):
        error = target_position - current_position
        error_dot = error - current_velocity
        control_signal = self.p_gain * error + self.d_gain * error_dot
        #control_signal = np.clip(control_signal, -self.max_veloicty, self.max_veloicty)
        return control_signal


class ForceFieldController:
    """Calculate Force Fields required to avoid collisions"""

    def __init__(self, dt: float, timesteps: int, p_gain: float, d_gain: float, max_velocity: float):
        self.dt = dt
        self.timesteps = timesteps

        self.base_controller = Controller(p_gain, d_gain, max_velocity)

    def predict_trajectory(self, current_position: np.ndarray, current_velocity: np.ndarray, target_position: np.ndarray, potential_field: np.ndarray = None):
        """Predict the trajectory of the robot based on the current position and velocity"""
        potential_field = np.zeros((current_position.shape[0], 2))
        number_of_shuttles = current_position.shape[0]

        if potential_field is None:
            potential_field = np.zeros((number_of_shuttles, 2))

        has_valid_trajectory = False
        # print(f'current_position: {current_position}')
        control_signal = np.zeros((number_of_shuttles, 2))
        current_position = current_position
        current_velocity = current_velocity
        while not has_valid_trajectory:
            for i in range(self.timesteps):
                for j in range(number_of_shuttles):
                    current_position[j][0:2], current_velocity[j][0:2]= self.new_states(current_position[j], current_velocity[j], target_position[j], potential_field[j])
                    if i == 0:
                        # Print shapes
                        control_signal[j,0:2] = current_velocity[j,0:2]

                # Check if there is a collision, and if so, calculate the force field
                collision, potentials = self.potential_field_checker(current_position, current_velocity, target_position, potential_field)
                potential_field += potentials

                if collision:
                    break

            if not collision:
                has_valid_trajectory = True
        #import time
        #time.sleep(10)
        #potential_field = np.clip(potential_field, -self.base_controller.max_veloicty, self.base_controller.max_veloicty)
        # control_signal = np.clip(control_signal, -self.base_controller.max_veloicty, self.base_controller.max_veloicty)
        # potential_field = np.clip(potential_field, -self.base_controller.max_veloicty, self.base_controller.max_veloicty)
        return control_signal, potential_field

    def new_states(self, current_position: np.ndarray, current_velocity: np.ndarray, target_position: np.ndarray, potential_field: np.ndarray = np.array([0.0, 0.0])):
        
        control_signal = self.base_controller.get_control_signal(current_position[0:2], target_position[0:2], current_velocity[0:2])
        #print(f'control_signal 1: {control_signal}')
        control_signal += potential_field
        #print(f'control_signal 2: {control_signal}')
        norm_of_control_signal = np.linalg.norm(control_signal)
        control_signal = (control_signal + potential_field) * norm_of_control_signal / np.linalg.norm(control_signal + potential_field) 

        next_velocity = control_signal  # current_velocity[0:2] + control_signal * self.dt
        next_position = current_position[0:2] + next_velocity * self.dt
        return next_position, next_velocity

    def potential_field_checker(self, current_position: np.ndarray, current_velocity: np.ndarray, target_position: np.ndarray, potential_field: np.ndarray):
        """Check if the potential field is valid"""
        # Paramters
        repulsive_gain = 0.003#0.003
        epsilon = 1e-5
        # Variables
        repulsive_force = np.zeros((current_position.shape[0], 2))
        collision = False
        for i in range(current_position.shape[0]):
            for j in range(current_position.shape[0]):
                if i != j:
                    distance = np.linalg.norm(current_position[i,0:2] - current_position[j,0:2], np.inf)
                    if i == 0 and j == 1:
                        # print(f'pos i: {current_position[i,0:2]}')
                        # print(f'pos j: {current_position[j,0:2]}')
                        #print(f'distance: {distance}')
                        pass
                        
                    if distance < 0.12:
                        #print(f'Collision between {i} and {j}')
                        collision = True
                        # Calculate the repulsive force
                        magnitude = repulsive_gain / (distance + epsilon)
                        np.clip(magnitude, -0.25, 0.25)
                        # magnitude = np.clip(magnitude, 0, 2)
                        # magnitude = repulsive_gain * (1 - distance) #/ distance
                        direction = (current_position[i, 0:2] - current_position[j, 0:2]) / distance
                        # Print shapes
                        repulsive_force[i] += magnitude * direction
        if not collision:
            return False, repulsive_force
        else:
            return True, repulsive_force