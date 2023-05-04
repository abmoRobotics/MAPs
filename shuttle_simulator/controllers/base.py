from abc import ABC, abstractmethod


class Controller(ABC):
    """Abstract class for controllers."""
    @abstractmethod
    def set_initial_position(self, position):
        pass

    @abstractmethod
    def set_initial_velocity(self, velocity):
        pass

    @abstractmethod
    def update(self, position, velocity, desired_position):
        pass

    @abstractmethod
    def get_control_signal(self, error):
        pass


class PotentialField(ABC):
    """Abstract class for potential fields."""
    @abstractmethod
    def calculate_force(self, position, velocity, desired_position):
        pass


class Predictor(ABC):
    """Abstract class for predictors."""
    @abstractmethod
    def predict(self, time: float):
        """Predicts the state of the system at a given time."""
        pass
