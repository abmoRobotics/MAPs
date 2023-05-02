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

def generate_random_goals(num_shuttles):
    desired_positions = np.random.rand(num_shuttles, 2)*6
    # Check if any of the desired positions are within a manhatten distance of 2 of each other
    invalid = np.sum(np.abs(desired_positions[:, None, :] - desired_positions[None, :, :]), axis=-1) < 2
    np.fill_diagonal(invalid, False)
    invalid = np.any(invalid)
    while invalid:
        desired_positions = np.random.rand(num_shuttles, 2)*6
        invalid = np.sum(np.abs(desired_positions[:, None, :] - desired_positions[None, :, :]), axis=-1) < 2
        np.fill_diagonal(invalid, False)
        invalid = np.any(invalid)

    return desired_positions

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
    #position = np.array([[1.0, 1.0], [6.0, 1.1], [1.0, 6.0], [6.0, 6.0]])
    # On a line
    # 6 shuttles
    position = np.array([[0.0, 1.0], [0.0, 2.5], [0.0, 4.0], [0.0, 5.5], [0.0, 7.0], [6.0, 1.0], [6.0, 2.5], [6.0, 4.0], [6.0, 5.5], [6.0, 7.0]])
    simulator = MultiShuttleSimulator(10, grid_size=GRID_SIZE, initial_positions=position)

    # attrative_potential_field = AttractivePotentialField(2, 0.01, 1)
    # repulsive_potential_field = RepulsivePotentialField(3, 0.01, 1)

    # simulator.get_shuttles()[0].set_controller(ForceFieldController(attrative_potential_field, repulsive_potential_field, 2))
    # simulator.get_shuttles()[1].set_controller(ForceFieldController(attrative_potential_field, repulsive_potential_field, 2))
    # simulator.get_shuttles()[2].set_controller(ForceFieldController(attrative_potential_field, repulsive_potential_field, 2))

    world = GridWorld(grid_size=CELL_SIZE, screen_size=SCREEN_SIZE, cell_size=CELL_SIZE)
    drawShuttle = DrawPlanarMotor(world=world, size=CELL_SIZE)

    # Set desired positions for testing
    #desired_positions = np.array([[6.0, 6.0], [1., 6.0], [6.0, 1.0], [1.0, 1.0]])
    # , [6.1, 1.0], [1.1, 1.0]])

    desired_positions = generate_random_goals(10)
    # desired_positions = [[2.0, 2.0], [2.0, 5.0], [0.0, 7.0]]

    # Set desired positions for testing
    for desired_position, shuttle in zip(desired_positions, simulator.shuttles):
        shuttle.set_desired_position(desired_position)
    potentials = np.zeros((len(simulator.shuttles), 2))
    prev_output = np.zeros((len(simulator.shuttles), 2))
    while True:
        # for shuttle in simulator.get_shuttles():
        #     shuttle.controller.set_other_shuttles_positions(simulator.get_other_shuttle_positions(shuttle.get_idx()))

        # simulator.get_shuttles()[0].controller.set_other_shuttles_positions(simulator.get_other_shuttle_positions(0))
        # simulator.get_shuttles()[1].controller.set_other_shuttles_positions(simulator.get_other_shuttle_positions(1))
        # simulator.get_shuttles()[2].controller.set_other_shuttles_positions(simulator.get_other_shuttle_positions(2))
        dt = clock.tick(120) / 1000  # Convert milliseconds to seconds
        relative_time += dt      # Update relative time
        print(f'dt: {dt}')
        world.update_display()   # Update the display
        predictor = ShuttlePredictor(copy.deepcopy(simulator.get_shuttles()), dt, 20)
        # print(f'Potential: {potentials}')
        potentials = potentials * 0.9
        output, potentials = predictor.predict(potentials)

        # Calculate angle between output and prev_output and check if difference is 10 degrees
        for idx, (o, p) in enumerate(zip(output, prev_output)):
            angle = np.arccos(np.dot(o, p) / (np.linalg.norm(o) * np.linalg.norm(p)))
            if angle > np.pi / 18 or angle < -np.pi / 18:
                output[idx] = (o + p) / 2

        for idx, (o, p) in enumerate(zip(output, prev_output)):
            # Scale amplitude of output to 90% or 110% of previous output
            if np.linalg.norm(p) > 1.1:
                if np.linalg.norm(o) > 1.1 * np.linalg.norm(p):
                    factor = np.linalg.norm(o) / np.linalg.norm(p)
                    output[idx] =  o / factor * 1.1

                if np.linalg.norm(o) < 0.9 * np.linalg.norm(p):
                    factor = np.linalg.norm(o) / np.linalg.norm(p)
                    output[idx] = o / factor * 0.9

        prev_output = output
        # Handle events
        for idx, shuttle in enumerate(simulator.get_shuttles()):
            drawShuttle.draw(screen, shuttle)
            drawShuttle.draw_arrow_to_goal(screen, shuttle)
            drawShuttle.draw_potential_vector(screen, shuttle, 10, output[idx])
            # shuttle.update(dt)
        pygame.display.flip()

        # draw potential vector

        for shuttle, output in zip(simulator.get_shuttles(), output):
            shuttle.update(dt=dt, control_signal=output)
        # for idx, potential in enumerate(potentials):
        #     print(f'Shuttle {idx} - potential: {potential}')

        # Check for collisions
        simulator.collision_detection()

        # Flip the display

        print(f'Current time: {relative_time:.2f} seconds')


if __name__ == "__main__":
    main()
