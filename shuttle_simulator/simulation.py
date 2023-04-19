from manipulators.manipulators import PlanarMotor
import numpy as np
import copy


class MultiShuttleSimulator:
    def __init__(self, num_shuttles, grid_size):
        self.shuttles = [PlanarMotor(position=np.array([i * 2., 0.]), idx=i, grid_size=grid_size) for i in range(num_shuttles)]
        self.initial_positions = [copy.deepcopy(shuttle.get_position()) for shuttle in self.shuttles]
        self.current_shuttle = 0

    def switch_shuttle(self, index):
        if 0 <= index < len(self.shuttles):
            self.current_shuttle = index

    def move_current_shuttle(self, direction):
        self.shuttles[self.current_shuttle].move(direction)

    def get_shuttles(self):
        return self.shuttles

    def _get_other_shuttles(self, index):
        return [shuttle for shuttle in self.shuttles if shuttle.get_idx() != index]

    def get_other_shuttle_positions(self, index):
        return [shuttle.get_position() for shuttle in self._get_other_shuttles(index)]

    def collision_detection(self):
        for i in range(len(self.shuttles)):
            for j in range(i + 1, len(self.shuttles)):
                # If circles overlap, return True

                if (self.shuttles[i].get_position()[0] - self.shuttles[j].get_position()[0]) ** 2 + (self.shuttles[i].get_position()[1] - self.shuttles[j].get_position()[1]) ** 2 < 1:
                    # Reset both shuttles to their initial positions
                    self.shuttles[i].set_position(copy.deepcopy(self.initial_positions[i]))
                    print(self.initial_positions)
                    self.shuttles[j].set_position(copy.deepcopy(self.initial_positions[j]))
                    print(f'collision detected between {i} and {j}')
        return False
