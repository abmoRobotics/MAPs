from controllers.base import Controller
from controllers.controllers import PDController
from .base import Manipulator, ManipulatorState
import numpy as np


class PlanarMotor(Manipulator):
    def __init__(self, position, idx, grid_size):
        self._idx = idx
        self.grid_size = grid_size
        # self.position = position
        self._velocity = np.array([0.0, 0.0])
        self._shuttle_state = PlanarMotorState(position=position, velocity=self._velocity)

        self.controller = PDController(p_gain=0.5, d_gain=0.01, max_velocity=1)
        self.controller.set_initial_position(position)
        self.controller.set_initial_velocity(self.get_velocity())

    def get_idx(self):
        return self._idx

    def update(self, dt: float, control_signal: np.ndarray = None):
        # Get control signal
        if control_signal is not None:
            self.set_velocity(control_signal)
        else:
            self.set_velocity(self.controller.get_control_signal(1))

        # Get current state
        current_position = self._shuttle_state.get_position()
        current_velocity = self._shuttle_state.get_velocity()

        # Update state
        self._shuttle_state.set_position(current_position + current_velocity * dt)

        # New state
        new_position = self._shuttle_state.get_position()
        new_velocity = self._shuttle_state.get_velocity()
        # Update controller
        self.controller.update(new_position, new_velocity, self.desired_position)

    def set_velocity(self, velocity):
        self._shuttle_state.set_velocity(velocity)

    def set_desired_position(self, desired_position):
        self.desired_position = desired_position

    def get_desired_position(self):
        return self.desired_position

    def get_state(self):
        return self._shuttle_state

    def set_state(self, state):
        self._shuttle_state = state

    def set_controller(self, controller: Controller):
        self.controller = controller
        self.controller.set_initial_position(self.get_position())
        self.controller.set_initial_velocity(self.get_velocity())

    def get_position(self):
        return self._shuttle_state.get_position()

    def get_velocity(self):
        return self._shuttle_state.get_velocity()

    def set_position(self, position):
        self._shuttle_state.set_position(position)

    def get_next_state(self, dt: float, current_state: ManipulatorState, additional_force: np.ndarray = np.array([0.0, 0.0])):
        """Get the next state of the planar motor given the current state and a time step."""
        # Get control signal
        self.controller.update(current_state.get_position(), current_state.get_velocity(), self.desired_position)

        self.controller.get_control_signal(1)
        # print(f'Control signal: {self.controller.get_control_signal(1)}')
        # print(f'Additional force: {additional_force}')
        # Update velocity
        norm = np.linalg.norm(self.controller.get_control_signal(1))
        force = (self.controller.get_control_signal(dt) + additional_force) * norm / np.linalg.norm(self.controller.get_control_signal(dt) + additional_force) 
        # print(norm)
        # print(force)
        # print(f'norm')
        current_state.set_velocity(force)
        # Update position
        current_state.set_position(current_state.get_position() + current_state.get_velocity() * dt)
        return current_state


class PlanarMotorState(ManipulatorState):
    """Class for storing the state of a planar motor."""

    def __init__(self, position: np.ndarray, velocity: np.ndarray):
        self._position = position
        self._velocity = velocity

    def set_position(self, position):
        """Set the position of the planar motor."""
        self._position = position

    def set_velocity(self, velocity):
        """Set the velocity of the planar motor."""
        self._velocity = velocity

    def get_position(self):
        """Get the position of the planar motor."""
        return self._position

    def get_velocity(self):
        """Get the velocity of the planar motor."""
        return self._velocity
