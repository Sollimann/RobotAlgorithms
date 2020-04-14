import pygame
from grid import OccupancyGridMap

# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
GRAY1 = (145, 145, 102)
GRAY2 = (77, 77, 51)
BLUE = (0, 0, 80)

colors = {
    0: WHITE,
    1: GREEN,
    255: GRAY1
}


class Animation:
    def __init__(self,
                 title="D* Lite Path Planning",
                 width=10,
                 height=10,
                 margin=0,
                 x_dim=100,
                 y_dim=50,
                 viewing_range=3):

        self.width = width
        self.height = height
        self.margin = margin
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.viewing_range = viewing_range

        pygame.init()

        # Set the 'width' and 'height' of the screen
        window_size = [(width + margin) * y_dim + margin,
                       (height + margin) * x_dim + margin]

        self.screen = pygame.display.set_mode(window_size)

        # create occupancy grid map
        """
        set initial values for the map occupancy grid
        |----------> x, column
        |       (x=2, y=0)
        |
        | (x=0, y=2)
        V
        y, row
        """
        self.grid_map = OccupancyGridMap(x_dim=x_dim,
                                         y_dim=y_dim,
                                         start_x=0,
                                         start_y=0,
                                         goal_x=x_dim - 2,
                                         goal_y=y_dim - 2)

        # Set title of screen
        pygame.display.set_caption(title)

        # set font
        pygame.font.SysFont('Comic Sans MS', 36)

        # Loop until the user clicks the close button
        self.done = False

        # used to manage how fast the screen updates
        self.clock = pygame.time.Clock()

    def draw_path(self):
        pass

    def run_game(self):
        cont = False
        while not self.done:

            for event in pygame.event.get():

                if event.type == pygame.QUIT:  # if user clicked close
                    print("quit")
                    self.done = True  # flag that we are done so we can exit loop
                elif (event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE) or cont:
                    # space bar pressed. call next action
                    print(self.grid_map.occupancy_grid_map)

                elif event.type == pygame.KEYDOWN and event.key == pygame.K_BACKSPACE:
                    print("backspace automates the press space")
                    cont = True

                elif pygame.mouse.get_pressed()[0]:
                    # User clicks the mouse. Get the position
                    (col, row) = pygame.mouse.get_pos()

                    # change the x/y screen coordinates to grid coordinates
                    x = row // (self.height + self.margin)
                    y = col // (self.width + self.margin)

                    # turn pos into cell
                    grid_cell = (x, y)

                    # set the location in the grid map
                    if not self.grid_map.is_occupied(grid_cell):
                        self.grid_map.set_obstacle(grid_cell)

            # set the screen background
            self.screen.fill(BLACK)

            # draw the grid
            for row in range(self.x_dim):
                for column in range(self.y_dim):
                    # color the cells
                    pygame.draw.rect(self.screen, colors[self.grid_map.occupancy_grid_map[row][column]],
                                     [(self.margin + self.width) * column + self.margin,
                                      (self.margin + self.height) * row + self.margin,
                                      self.width,
                                      self.height])

            # fill in the goal cell with green
            pygame.draw.rect(self.screen, GREEN, [(self.margin + self.width) * self.grid_map.goal[1] + self.margin,
                                                  (self.margin + self.height) * self.grid_map.goal[0] + self.margin,
                                                  self.width,
                                                  self.height])

            # draw a moving robot, based on current coordinates
            robot_center = [round(self.grid_map.current[1] * (self.width + self.margin) + self.width / 2) + self.margin,
                            round(
                                self.grid_map.current[0] * (self.height + self.margin) + self.height / 2) + self.margin]

            # draw robot position as red circle
            pygame.draw.circle(self.screen, RED, robot_center, round(self.width / 2) - 2)

            # draw robot local grid map (viewing range)
            pygame.draw.rect(self.screen, BLUE, [robot_center[1] - self.viewing_range * (self.width + self.margin),
                                                 robot_center[0] - self.viewing_range * (self.height + self.margin),
                                                 2 * self.viewing_range * (self.width + self.margin),
                                                 2 * self.viewing_range * (self.height + self.margin)], 2)

            # set game tick
            self.clock.tick(20)

            # go ahead and update screen with that we've drawn
            pygame.display.flip()

        # be 'idle' friendly. If you forget this, the program will hang on exit
        pygame.quit()
