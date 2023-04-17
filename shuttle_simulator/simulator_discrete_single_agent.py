import pygame
import sys


# Initialize pygame
pygame.init()

# Define constants
GRID_SIZE = (10, 10)
CELL_SIZE = 50
WINDOW_SIZE = (GRID_SIZE[0] * CELL_SIZE, GRID_SIZE[1] * CELL_SIZE)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

class PlanarMotor:
    def __init__(self, position):
        self.position = position
        self.velocity = 0
        self.acceleration = 0

    def move(self, direction):
        x, y = self.position
        dx, dy = direction
        self.position = (max(min(x + dx, GRID_SIZE[0] - 1), 0), max(min(y + dy, GRID_SIZE[1] - 1), 0))

    def draw(self, screen):
        x, y = self.position
        pygame.draw.rect(screen, BLACK, (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE))


def draw_grid(screen):
    for x in range(0, WINDOW_SIZE[0], CELL_SIZE):
        py = pygame.draw.line(screen, BLACK, (x, 0), (x, WINDOW_SIZE[1]))
    for y in range(0, WINDOW_SIZE[1], CELL_SIZE):
        px = pygame.draw.line(screen, BLACK, (0, y), (WINDOW_SIZE[0], y))

def main():
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("Shuttle Simulator")
    clock = pygame.time.Clock()

    motor = PlanarMotor((0, 0))

    while True:
        screen.fill(WHITE)
        draw_grid(screen)
        motor.draw(screen)
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    motor.move((-1, 0))
                elif event.key == pygame.K_RIGHT:
                    motor.move((1, 0))
                elif event.key == pygame.K_UP:
                    motor.move((0, -1))
                elif event.key == pygame.K_DOWN:
                    motor.move((0, 1))

        clock.tick(30)

if __name__ == "__main__":
    main()