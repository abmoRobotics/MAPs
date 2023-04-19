import pygame
import logging
from controllers.controllers import ShuttlePredictor    # ForceFieldController, AttractivePotentialField, RepulsivePotentialField
from worlds import GridWorld, DrawPlanarMotor
from simulation import MultiShuttleSimulator
import numpy as np
import copy

# Set logging level from the following: DEBUG, INFO, WARNING, ERROR, CRITICAL
logging.basicConfig(level=logging.WARNING)


# Define debug decorator
def log_decorator(func):
    def wrapper(*args, **kwargs):
        result = func(*args, **kwargs)
        logging.info(f"{func.__name__} returned {result}")
        return result
    return wrapper


# Initialize pygame
pygame.init()

# Define constants
GRID_SIZE = (6, 8)
CELL_SIZE = 150
SCREEN_SIZE = (GRID_SIZE[0] * CELL_SIZE, GRID_SIZE[1] * CELL_SIZE)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
SPEED = 1 / 60


def main():
    screen = pygame.display.set_mode(SCREEN_SIZE)
    pygame.display.set_caption("Continuous Multi-Shuttle Simulator")
    clock = pygame.time.Clock()
    # import time
    # time.sleep(1)
    # controller_gain = 1.1
    relative_time = 0
    simulator = MultiShuttleSimulator(3, grid_size=GRID_SIZE)

    # attrative_potential_field = AttractivePotentialField(2, 0.01, 1)
    # repulsive_potential_field = RepulsivePotentialField(3, 0.01, 1)

    # simulator.get_shuttles()[0].set_controller(ForceFieldController(attrative_potential_field, repulsive_potential_field, 2))
    # simulator.get_shuttles()[1].set_controller(ForceFieldController(attrative_potential_field, repulsive_potential_field, 2))
    # simulator.get_shuttles()[2].set_controller(ForceFieldController(attrative_potential_field, repulsive_potential_field, 2))

    world = GridWorld(grid_size=CELL_SIZE, screen_size=SCREEN_SIZE, cell_size=CELL_SIZE)
    drawShuttle = DrawPlanarMotor(world=world, size=CELL_SIZE)

    # Set desired positions for testing
    desired_positions = np.array([[6.0, 5.0], [2.0, 5.0], [0.0, 7.0]])
    # desired_positions = [[2.0, 2.0], [2.0, 5.0], [0.0, 7.0]]

    # Set desired positions for testing
    for desired_position, shuttle in zip(desired_positions, simulator.shuttles):
        shuttle.set_desired_position(desired_position)

    while True:
        # for shuttle in simulator.get_shuttles():
        #     shuttle.controller.set_other_shuttles_positions(simulator.get_other_shuttle_positions(shuttle.get_idx()))

        # simulator.get_shuttles()[0].controller.set_other_shuttles_positions(simulator.get_other_shuttle_positions(0))
        # simulator.get_shuttles()[1].controller.set_other_shuttles_positions(simulator.get_other_shuttle_positions(1))
        # simulator.get_shuttles()[2].controller.set_other_shuttles_positions(simulator.get_other_shuttle_positions(2))
        dt = clock.tick(60) / 1000  # Convert milliseconds to seconds
        relative_time += dt      # Update relative time
        world.update_display()   # Update the display

        # Handle events
        for shuttle in simulator.get_shuttles():
            drawShuttle.draw(screen, shuttle)
            #shuttle.update(dt)
        pygame.display.flip()
        predictor = ShuttlePredictor(copy.deepcopy(simulator.get_shuttles()), dt, 10)
        potentials = predictor.predict()
        for shuttle, potential in zip(simulator.get_shuttles(), potentials):
            shuttle.update(dt=dt, control_signal=potential)
        # for idx, potential in enumerate(potentials):
        #     print(f'Shuttle {idx} - potential: {potential}')

        # Check for collisions
        simulator.collision_detection()

        # Flip the display
        

        print(f'Current time: {relative_time:.2f} seconds')


if __name__ == "__main__":
    main()
