from controllers import Controller, PDController, ForceFieldController

class PlanarMotor:
    def __init__(self, position, idx, grid_size):
        self.idx = idx
        self.grid_size = grid_size
        self.position = position
        self.velocity = [0, 0]
        self.controller = PDController(10, 0.1, 2)
        self.controller.set_initial_position(position)
        self.controller.set_initial_velocity(self.velocity)

    def update(self, dt):
        # position_error = [desired_position - current_position for desired_position, current_position in zip(self.desired_position, self.position)]

        self.velocity = self.controller.get_control_signal(1)
        # Update position
        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt
        self.position[0] = max(min(self.position[0], self.grid_size[0] - 1), 0)
        self.position[1] = max(min(self.position[1], self.grid_size[1] - 1), 0)
        # Update controller
        self.controller.update(self.position, self.velocity, self.desired_position)

    def set_velocity(self, velocity):
        self.velocity = velocity

    def set_desired_position(self, desired_position):
        self.desired_position = desired_position

    def set_controller(self, controller: Controller):
        self.controller = controller
        self.controller.set_initial_position(self.position)
        self.controller.set_initial_velocity(self.velocity)
