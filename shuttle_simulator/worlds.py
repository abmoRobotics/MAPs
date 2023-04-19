from abc import ABC, abstractmethod
from manipulators.manipulators import PlanarMotor
import pygame


class World(ABC):
    def __init__(self):
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)

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
