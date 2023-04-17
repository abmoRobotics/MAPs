import pygame
import sys

# Initialize pygame
pygame.init()

# Define constants
GRID_SIZE = (6, 8)
CELL_SIZE = 120
WINDOW_SIZE = (GRID_SIZE[0] * CELL_SIZE, GRID_SIZE[1] * CELL_SIZE)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)


class PlanarMotor:
    def __init__(self, position, idx):
        self.idx = idx
        self.position = position

    def move(self, direction):
        x, y = self.position
        dx, dy = direction
        self.position = (max(min(x + dx, GRID_SIZE[0] - 1), 0), max(min(y + dy, GRID_SIZE[1] - 1), 0))

    def draw(self, screen):
        x, y = self.position
        idx = self.idx
        pygame.draw.rect(screen, BLACK, (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE))
        # Print idx on top of the shuttle
        font = pygame.font.SysFont('Arial', 30)
        text = font.render(str(idx), True, (255, 0, 0))
        # Center the text in the cell
        text_rect = text.get_rect(center=(x * CELL_SIZE + CELL_SIZE / 2, y * CELL_SIZE + CELL_SIZE / 2))
        screen.blit(text, text_rect)


def draw_grid(screen):
    for x in range(0, WINDOW_SIZE[0], CELL_SIZE):
        py = pygame.draw.line(screen, BLACK, (x, 0), (x, WINDOW_SIZE[1]))
    for y in range(0, WINDOW_SIZE[1], CELL_SIZE):
        px = pygame.draw.line(screen, BLACK, (0, y), (WINDOW_SIZE[0], y))

class MultiShuttleSimulator:
    def __init__(self, num_shuttles):
        self.shuttles = [PlanarMotor(position=(i, 0), idx=i) for i in range(num_shuttles)]
        self.current_shuttle = 0

    def switch_shuttle(self, index):
        if 0 <= index < len(self.shuttles):
            self.current_shuttle = index

    def move_current_shuttle(self, direction):  
        self.shuttles[self.current_shuttle].move(direction)

    def draw(self, screen):
        for shuttle in self.shuttles:
            shuttle.draw(screen)


def main():
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("Multi-Shuttle Simulator")
    clock = pygame.time.Clock()

    simulator = MultiShuttleSimulator(3)

    while True:
        screen.fill(WHITE)
        draw_grid(screen)
        simulator.draw(screen)
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    simulator.move_current_shuttle((-1, 0))
                elif event.key == pygame.K_RIGHT:
                    simulator.move_current_shuttle((1, 0))
                elif event.key == pygame.K_UP:
                    simulator.move_current_shuttle((0, -1))
                elif event.key == pygame.K_DOWN:
                    simulator.move_current_shuttle((0, 1))
                elif event.key == pygame.K_1:
                    simulator.switch_shuttle(0)
                elif event.key == pygame.K_2:
                    simulator.switch_shuttle(1)
                elif event.key == pygame.K_3:
                    simulator.switch_shuttle(2)
            
        
        clock.tick(30)

if __name__ == "__main__":
    main()