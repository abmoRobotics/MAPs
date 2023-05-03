from abc import ABC, abstractmethod
from controllers.controllers import Controller
import numpy as np
from typing import List


class ManipulatorState(ABC):
    """Abstract class for storing the state of a manipulator"""

    @abstractmethod
    def set_position(self, position):
        pass

    @abstractmethod
    def set_velocity(self, velocity):
        pass

    @abstractmethod
    def get_position(self):
        pass

    @abstractmethod
    def get_velocity(self):
        pass

    # @abstractmethod
    # def get_state(self):
    #     pass

    # @abstractmethod
    # def set_state(self, state):
    #     pass


class Manipulator(ABC):
    """Abstract class for manipulators."""

    @abstractmethod
    def get_idx(self):
        pass

    @abstractmethod
    def update(self, dt):
        pass

    @abstractmethod
    def set_velocity(self, velocity):
        pass

    @abstractmethod
    def set_desired_position(self, desired_position):
        pass

    @abstractmethod
    def set_controller(self, controller: Controller):
        pass

    @abstractmethod
    def get_state(self):
        pass

    @abstractmethod
    def set_state(self, state: ManipulatorState):
        pass

    @abstractmethod
    def get_next_state(self, dt: float, current_state: ManipulatorState, additional_force: np.ndarray):
        pass

    # @abstractmethod
    # def simulate_next_step(self, state, dt):
    #     pass


class ManipulatorTemporalStates(ABC):
    """Abstract class for storing the temporal states of a manipulator"""

    @property
    def __init__(self, manipulators: List[Manipulator], horizon) -> None:
        number_of_manipulators = len(manipulators)
        self._temporal_states = np.array([number_of_manipulators, horizon], dtype=ManipulatorState)
