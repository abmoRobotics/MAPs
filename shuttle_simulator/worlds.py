from abc import ABC, abstractmethod
from manipulators.manipulators import PlanarMotor
import pygame
import numpy as np
import math
class World(ABC):
    def __init__(self):
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.yellow = (255, 255, 0)

    @abstractmethod
    def initialize(self):
        pass

    @abstractmethod
    def update_display(self, dt):
        pass


class GridWorld(World):
    def __init__(self, grid_size: tuple, screen_size: tuple, cell_size: int) -> None:
        super().__init__()
        self.grid_size = grid_size
        self.screen_size = screen_size

        self.cell_size = cell_size
        self.initialize()

    def initialize(self):
        self.screen = pygame.display.set_mode(self.screen_size)
        pygame.display.set_caption("Grid World")

    def update_display(self):
        self.screen.fill(self.white)
        self._draw_grid()
        # pygame.display.flip()

    def _draw_grid(self):
        for x in range(0, self.screen_size[0], self.cell_size):
            pygame.draw.line(self.screen, self.black, (x, 0), (x, self.screen_size[1]))
        for y in range(0, self.screen_size[1], self.cell_size):
            pygame.draw.line(self.screen, self.black, (0, y), (self.screen_size[0], y))


class DrawPlanarMotor():
    def __init__(self, world: World, size: int):
        self.world = world
        self.size = size

    def draw(self, screen: pygame.display, shuttle: PlanarMotor):
        x, y = shuttle.get_position()
        idx = shuttle.get_idx()
        pygame.draw.rect(screen, self.world.black, (x * self.size, y * self.size, self.size, self.size))
        # Print idx on top of the shuttle
        font = pygame.font.SysFont('Arial', 30)
        text = font.render(str(idx), True, self.world.red)
        # Center the text in the cell
        text_rect = text.get_rect(center=(x * self.size + self.size / 2, y * self.size + self.size / 2))
        screen.blit(text, text_rect)

    def draw_arrow_to_goal(self, screen: pygame.display, shuttle: PlanarMotor, arrow_size: int = 10):

        pygame.draw.line(screen, self.world.red, shuttle.get_position() * self.size + self.size / 2, shuttle.desired_position * self.size + self.size / 2, 2)
        pos1 = shuttle.get_position()
        dx = shuttle.desired_position[0] - shuttle.get_position()[0]
        dy = shuttle.desired_position[1] - shuttle.get_position()[1]
        angle = math.atan2(dy, dx)

    def draw_potential_vector(self, screen: pygame.display, shuttle: PlanarMotor, arrow_size: int = 10, potential: np.ndarray = None):
        if potential is None:
            potential = shuttle
            # Draw line from current position to potential vector
        pygame.draw.line(screen, self.world.green, shuttle.get_position() * self.size + self.size / 2, (shuttle.get_position() + potential) * self.size + self.size / 2, 2)


        # # Draw arrow head
        # arrow_tip_x = (shuttle.desired_position[0] - arrow_size * np.cos(angle))
        # arrow_tip_y = (shuttle.desired_position[1] - arrow_size * np.sin(angle))

        # arrow_point1 = (arrow_tip_x + arrow_size * math.sin(angle), arrow_tip_y - arrow_size * math.cos(angle))
        # arrow_point2 = (arrow_tip_x - arrow_size * math.sin(angle), arrow_tip_y + arrow_size * math.cos(angle))

        # pygame.draw.line(screen, self.world.red, shuttle.desired_position * self.size, arrow_point1, 2)
        # pygame.draw.line(screen, self.world.red, shuttle.desired_position * self.size, arrow_point2, 2)
