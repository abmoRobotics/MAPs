from abc import ABC, abstractmethod
import numpy as np

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

    def get_control_signal(self,  error):
        # Compute error
        position_error = [desired_position - current_position for desired_position, current_position in zip(self.desired_position, self.position)]
        velocity_error = [desired_velocity - current_velocity for desired_velocity, current_velocity in zip(position_error, self.velocity)]
        # Compute control signal
        control_signal = [self.p_gain * position_error + self.d_gain * velocity_error for position_error, velocity_error in zip(position_error, velocity_error)]
        # Clip control signal
        control_signal = [max(min(control_signal, self.max_velocity), -self.max_velocity) for control_signal in control_signal]
        return control_signal
    


class PotentialField(ABC):
    """Abstract class for potential fields."""
    @abstractmethod
    def calculate_force(self, position, velocity, desired_position):
        pass


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
                    force_magnitude = self.repulsive_gain / (distance**2+0.00001)
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


# class ForceFieldController(Controller):
#     """Force Field Controller."""
#     def __init__(self, p_gain: float, d_gain: float, max_force: float):
#         """ Constructor

#         Args:
#             p_gain: Proportional gain
#             d_gain: Derivative gain
#             max_force: Maximum force
#         """
#         self.p_gain = p_gain
#         self.d_gain = d_gain
#         self.max_force = max_force

#     def set_initial_position(self, position):
#         self.position = np.array(position)
#         self.desired_position = np.array(position)

#     def set_initial_velocity(self, velocity):
#         self.velocity = np.array(velocity)

#     def update(self, position, velocity, desired_position):
#         self.position = np.array(position)
#         self.velocity = np.array(velocity)
#         self.desired_position = np.array(desired_position)

#     def get_control_signal(self, error):
#         # Compute error
#         position_error = self.desired_position - self.position
#         velocity_error = -self.velocity

#         # Compute control signal (force)
#         control_signal = self.p_gain * position_error + self.d_gain * velocity_error

#         # Clip control signal (force)
#         control_signal_magnitude = np.linalg.norm(control_signal)
#         if control_signal_magnitude > self.max_force:
#             control_signal = (control_signal / control_signal_magnitude) * self.max_force

#         return control_signal

