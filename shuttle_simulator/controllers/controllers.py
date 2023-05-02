
from typing import List
import numpy as np

from .base import Controller, PotentialField, Predictor
from manipulators.base import Manipulator, ManipulatorState
import logging
from typing import Tuple
import copy


class PDController(Controller):
    """PD Controller."""
    def __init__(self, p_gain: float, d_gain: float, max_velocity: float):
        """ Constructor

        Args:
            p_gain: Proportional gain
            d_gain: Derivative gain
            max_velocity: Maximum velocity
        """
        self.p_gain = p_gain
        self.d_gain = d_gain
        self.max_velocity = max_velocity

    def set_initial_position(self, position):
        self.position = position
        self.desired_position = position

    def set_initial_velocity(self, velocity):
        self.velocity = velocity

    def update(self, position, velocity, desired_position):
        self.position = position
        self.velocity = velocity
        self.desired_position = desired_position

    def get_control_signal(self, error):
        # Compute error
        position_error = np.array([desired_position - current_position for desired_position, current_position in zip(self.desired_position, self.position)])
        velocity_error = np.array([desired_velocity - current_velocity for desired_velocity, current_velocity in zip(position_error, self.velocity)])
        # Compute control signal
        control_signal = np.array([self.p_gain * position_error + self.d_gain * velocity_error for position_error, velocity_error in zip(position_error, velocity_error)])
        # Clip control signal
        control_signal = np.array([max(min(control_signal, self.max_velocity), -self.max_velocity) for control_signal in control_signal])
        return control_signal


class AttractivePotentialField(PotentialField):

    def __init__(self, p_gain: float, d_gain: float, max_force: float):
        """ Constructor

        Args:
            p_gain: Proportional gain
            d_gain: Derivative gain
        """
        self.p_gain = p_gain
        self.d_gain = d_gain

    def calculate_force(self, position, velocity, desired_position):
        position_error = desired_position - position
        velocity_error = -velocity
        return self.p_gain * position_error + self.d_gain * velocity_error


class RepulsivePotentialField(PotentialField):
    """Repulsive potential field."""
    def __init__(self, p_gain: float, d_gain: float, max_force: float):
        """ Constructor

        Args:
            p_gain: Proportional gain
            d_gain: Derivative gain
        """
        self.p_gain = p_gain
        self.d_gain = d_gain
        self.repulsive_range = 2
        self.repulsive_gain = self.p_gain

    def calculate_force(self, position, other_shuttles_positions):
        repulsive_force = np.zeros_like(position)
        for other_position in other_shuttles_positions:
            other_position = np.array(other_position)
            distance_vector = position - other_position
            distance = np.linalg.norm(distance_vector)
            if distance < self.repulsive_range:
                force_magnitude = self.repulsive_gain / (distance**2 + 0.00001)
                force_direction = distance_vector / distance
                repulsive_force += force_magnitude * force_direction
        return repulsive_force


class ForceFieldController(Controller):
    def __init__(self, attractive_field, repulsive_field, max_force):
        self.attractive_field = attractive_field
        self.repulsive_field = repulsive_field
        self.max_force = max_force

    def set_initial_position(self, position):
        self.position = np.array(position)
        self.desired_position = np.array(position)

    def set_initial_velocity(self, velocity):
        self.velocity = np.array(velocity)

    def update(self, position, velocity, desired_position):
        self.position = np.array(position)
        self.velocity = np.array(velocity)
        self.desired_position = np.array(desired_position)

    def get_control_signal(self, error):
        attractive_force = self.attractive_field.calculate_force(self.position, self.velocity, self.desired_position)
        repulsive_force = self.repulsive_field.calculate_force(self.position, self.get_other_shuttles_positions())
        control_signal = attractive_force + repulsive_force

        control_signal_magnitude = np.linalg.norm(control_signal)
        if control_signal_magnitude > self.max_force:
            control_signal = (control_signal / control_signal_magnitude) * self.max_force

        return control_signal

    def set_other_shuttles_positions(self, other_shuttles_positions):
        self.other_shuttles_positions = other_shuttles_positions

    def get_other_shuttles_positions(self):
        return self.other_shuttles_positions


class ShuttlePredictor(Predictor):
    """ Class for predicting future states of shuttles, and add potential to the states if they are in collision with other shuttles.

    Args:
        shuttles: List of shuttles
        dt: Time step
        timesteps: Number of time steps to predict

    Attributes:
        predicted_states: List of predicted states
        predicted_times: List of predicted times
        shuttles: List of shuttles
        states: List of current states
        """
    def __init__(self, shuttles: List[Manipulator], dt: float, timesteps: int):
        """ShuttlePredictor constructor"""
        # self.predicted_times = [dt * i for i in range(timesteps)]   # Initialize predicted times
        self.dt = dt
        self.time_range = timesteps
        self.shuttles = shuttles
        self.states: List[ManipulatorState] = [shuttle.get_state() for shuttle in shuttles]

    def predict(self, potentials: np.ndarray = None) -> np.ndarray:
        """ Predict future states of shuttles, and add potential to the states if they are in collision with other shuttles."""

        # Initialize variables
        is_finished = False

        if potentials is None:
            potentials = np.zeros((len(self.shuttles), 2))

        first_control_signal = np.zeros((len(self.shuttles), 2))
        counter = 0
        # Predict future states until no collisions detected
        while not is_finished:
            # Get attributes of the class
            time_range = copy.deepcopy(self.time_range)
            states = copy.deepcopy(self.states)
            collision = False

            # Predict future states
            for i in range(time_range):
                for idx, shuttle in enumerate(self.shuttles):
                    current_state = states[idx]                         # Get current state of the shuttle
                    new_state = shuttle.get_next_state(dt=self.dt, current_state=current_state, additional_force=potentials[idx])  # Get next state of the shuttle
                    states[idx] = new_state                       # Update the current state of the shuttle
                    if i == 0:
                        direction_vector = shuttle.get_desired_position() - new_state.get_position()
                        velocity_magnitude = np.linalg.norm(new_state.get_velocity()) * 0.25
                        velocity_command = new_state.get_velocity()# + direction_vector * velocity_magnitude
                        first_control_signal[idx] = velocity_command

                # Check if there is a collision, and calculate potential field.
                collision, potential = self.potentialFieldChecker(self.shuttles, states)
                potentials += potential
                

                # If collision detected, then stop predicting, and add potential to the states, and try again with different potential field
                if collision:
                    # import time
                    
                    # print(f'Collision detected at time {i} for shuttle {shuttle.get_idx()}')
                    # time.sleep(2)
                    # logging.info(f'Collision detected at time {i} for shuttle {shuttle.get_idx()}')
                    break

            # If no collisions detected for all the shuttles at the end of the time range, then the prediction is finished
            if not collision:
                is_finished = True
 
        counter += 1
        #print(counter)
        print(f'Potentials: {potentials[0]}')
   #     print(f) 
        return first_control_signal, potentials

    def potentialFieldChecker(self, shuttles, states: List[ManipulatorState]) -> Tuple[bool, np.ndarray]:
        """ Check if there is a collision between shuttles, and calculate the potential field.

        Args:
            shuttles: List of shuttles
            states: List of states of the shuttles

        Returns:
            collision: Boolean indicating if there is a collision between shuttles
            potential: Repulsive potential field
        """
        repulsive_gain = 1
        repulsive_force = np.zeros((len(shuttles), 2), dtype=float)
        collision = False
        # Check if there is a collision between shuttles
        for i in range(len(shuttles)):
            for j in range(len(shuttles)):
                if i != j:
                    # Calculate the distance between the shuttles, using the infinite norm, and check if it is less than 2
                    pos1 = states[i].get_position()
                    pos2 = states[j].get_position()
                    L_inifnite_norm = np.linalg.norm(states[i].get_position() - states[j].get_position(), np.inf)
                    if L_inifnite_norm < 1:
                        collision = True
                        # If there is a collision, then calculate the repulsive force
                        force_magnitude = repulsive_gain / (L_inifnite_norm**2 + 0.00001)
                        #force_magnitude = np.clip(force_magnitude, -2, 2)
                        force_direction = (states[i].get_position() - states[j].get_position()) / L_inifnite_norm
                        repulsive_force[i] += force_magnitude * force_direction

        
        # If there is no collision, then return False, and 0 as the potential
        if not collision:
            return False, repulsive_force
        else:
            return True, repulsive_force


if __name__ == '__main__':
    pass
