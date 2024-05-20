import pygame
import numpy as np
import matplotlib.pyplot as plt
import heapq

# Define color
red = (200, 0, 0)
blue = (0, 0, 255)
green = (0, 155, 0)
yellow = (155, 155, 0)
white = (255, 255, 255)
black = (0, 0, 0)
grey  = (128, 128, 128)

# Window size
WIDTH = 800
HEIGHT = 800

# Lengths of leg parts: 
a1 = 300
a2 = 300

class Robot:
    def __init__(self):
        self.mx = 0
        self.my = 0
        self.orientation = 0
        self.x = 0
        self.y = 0
        self.x2 = 0
        self.y2 = 0
        self.x_data = []
        self.y_data = []
        self.q1_data = []
        self.q2_data = []
        self.path_points = []  # Store path points for movement

    def set(self, mx, my):
        self.mx = mx
        self.my = my
        
    def data_analyzing(self, screen, q1_lims=[-3, -1.55], q2_lims=[1, 1.85], N=5):
        q1_s = np.linspace(q1_lims[0], q1_lims[1], N)
        q2_s = np.linspace(q2_lims[0], q2_lims[1], N)    
        for q1 in q1_s:
            x_1 = np.cos(q1) * a1
            y_1 = np.sin(q1) * a1
            for q2 in q2_s:
                x_2 = np.cos(q2) * a1 + x_1
                y_2 = np.sin(q2) * a1 + y_1
                pygame.draw.circle(screen, blue, (x_2, y_2), 3, width=3)
                self.x_data.append(np.abs(x_2))
                self.y_data.append(np.abs(y_2))
                self.q1_data.append(q1*180/np.pi)
                self.q2_data.append(q2*180/np.pi)
                
    def movement(self):   
        grid = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ]
        cell_width = WIDTH/len(grid)
        cell_height = HEIGHT/len(grid[0])
        src = [7, 8]
        dest_list = ([5, 3], [7, 2], [7, 8])
        
        for dest in dest_list:
            print(f"Searching path to destination: {dest}")
            a_star = AStarSearch(grid, src, dest)
            a_star.a_star_search()
            path = a_star.trace_path()
            src = dest
            for i in path:
                vertical = cell_width * (i[1]-.5)
                horizontal = cell_height * (i[0]-.5)
                print(f'vertical: {vertical}; horizontal: {horizontal}')
                self.path_points.append((vertical, horizontal))  # Collect path points
                

    def draw(self, screen):
        robot.movement()
        if self.path_points:
            self.mx, self.my = self.path_points.pop(0)  # Update position to the next path point

        C = ((WIDTH/2 - self.mx)**2 + (.15 * HEIGHT - self.my)**2 - a1**2 - a2**2) / (2 * a1 * a2)
        font = pygame.font.Font(None, 36)
        
        if -1 <= C <= 1:  # Only calculate q1 and q2 if the target is reachable
            # Inverse Kinematics
            q2 = np.arccos(C)
            q1 = np.arctan2((.15 * HEIGHT - self.my), (WIDTH / 2 - self.mx)) - np.arctan2(a2 * np.sin(q2), a1 + a2 * np.cos(q2))
            self.y = a1 * np.sin(q1)
            self.x = a1 * np.cos(q1)
            self.y2 = a2 * np.sin(q2) + self.y
            self.x2 = a2 * np.cos(q2) + self.x
            
            # Drawing pieslice for angles
            pygame.draw.arc(screen, grey, (WIDTH / 2 + .30 * a1 * np.sqrt(2) / 2 - 50, .15 * HEIGHT - .30 * a1 * np.sqrt(2) / 2 - 50, 100, 100), 1.5 * np.pi, np.pi - q1, 2)
            textq1 = font.render(f'q1: {round((0.5 * np.pi + q1) * -180 / np.pi, 1)}°', True, grey)
            screen.blit(textq1, (WIDTH / 2 + .30 * a1 * np.sqrt(2) / 2 + 40, .15 * HEIGHT - .48 * a1 * np.sqrt(2) / 2 - 3))
            
            pygame.draw.arc(screen, grey, (WIDTH / 2 - 50, .15 * HEIGHT - 50, 100, 100), .5 * np.pi, np.pi - (q1 + q2), 3)
            textq2 = font.render(f'q2: {round((-0.5 * np.pi + q1 + q2) * -180 / np.pi, 1)}°', True, grey)
            screen.blit(textq2, (WIDTH / 2 - 170, HEIGHT * 0.11 - 3))
            text = font.render("Target is reachable!", True, black)
            # Thigh upper 
            pygame.draw.line(screen, black, (WIDTH / 2, HEIGHT * 0.15), (WIDTH / 2 - self.x, .15 * HEIGHT - self.y), 4)
            # Calf
            pygame.draw.line(screen, black, (WIDTH / 2 - self.x, .15 * HEIGHT - self.y), (self.mx, self.my), 4)
            # Transition top1
            pygame.draw.line(screen, black, (WIDTH / 2 + .30 * a1 * np.sqrt(2) / 2, .15 * HEIGHT - .30 * a1 * np.sqrt(2) / 2), (WIDTH / 2 + .30 * a1 * (np.sqrt(2) / 2 - np.cos(q1)), .15 * HEIGHT - .30 * a1 * (np.sqrt(2) / 2 + np.sin(q1))), 4)
            # Transition top2
            pygame.draw.line(screen, black, (WIDTH / 2 + .30 * a1 * (np.sqrt(2) / 2 - np.cos(q1)), .15 * HEIGHT - .30 * a1 * (np.sqrt(2) / 2 + np.sin(q1))), ((WIDTH / 2 - self.x * .30), .15 * HEIGHT - self.y * .30), 4)       
            # Transition bottom
            pygame.draw.line(screen, black, (WIDTH / 2, HEIGHT * 0.15), (WIDTH / 2 - .30 * a1 * np.cos(q1 + q2), HEIGHT * 0.15 - .30 * a1 * np.sin(q1 + q2)), 4)
            # Thigh lower    
            pygame.draw.line(screen, black, (WIDTH / 2 - .30 * a1 * np.cos(q1 + q2), HEIGHT * 0.15 - .30 * a1 * np.sin(q1 + q2)), (.7 * (WIDTH / 2 - self.x) + .30 * self.mx, .7 * (.15 * HEIGHT - self.y) + .30 * self.my), 4)
            
            # Servo 1 
            pygame.draw.circle(screen, red, (WIDTH / 2 + .30 * a1 * np.sqrt(2) / 2, .15 * HEIGHT - .30 * a1 * np.sqrt(2) / 2), 10, width=10)
            # Servo 2
            pygame.draw.circle(screen, red, (WIDTH / 2, HEIGHT * 0.15), 10, width=10)
            
            # Aesthetic thing
            pygame.draw.circle(screen, black, (WIDTH / 2 - self.x, .15 * HEIGHT - self.y), 5, width=5)
            pygame.draw.circle(screen, black, (self.mx, self.my), 5, width=5)
            pygame.draw.circle(screen, black, (WIDTH / 2 + .30 * a1 * (np.sqrt(2) / 2 - np.cos(q1)), .15 * HEIGHT - .30 * a1 * (np.sqrt(2) / 2 + np.sin(q1))), 5, width=5)
            pygame.draw.circle(screen, black, ((WIDTH / 2 - self.x * .30), .15 * HEIGHT - self.y * .30), 5, width=5)
            pygame.draw.circle(screen, black, (WIDTH / 2 - .30 * a1 * np.cos(q1 + q2), HEIGHT * 0.15 - .30 * a1 * np.sin(q1 + q2)), 5, width=5)
            pygame.draw.circle(screen, black, (.7 * (WIDTH / 2 - self.x) + .30 * self.mx, .7 * (.15 * HEIGHT - self.y) + .30 * self.my), 5, width=5)    
            s1name = font.render("S1", True, black)     
            screen.blit(s1name, (WIDTH / 2 + .30 * a1 * np.sqrt(2) / 2, .15 * HEIGHT - .48 * a1 * np.sqrt(2) / 2))
            s2name = font.render("S2", True, black)     
            screen.blit(s2name, (WIDTH / 2, HEIGHT * 0.11))
            
        else:
            text = font.render("Target is not reachable!", True, black)
        
        screen.blit(text, (0, 20))

















class AStarSearch:
    def __init__(self, grid, src, dest):
        self.ROW = len(grid)
        self.COL = len(grid[0])
        self.grid = grid
        self.src = src
        self.dest = dest
        self.cell_details = [[self.Cell() for _ in range(self.COL)] for _ in range(self.ROW)]

    class Cell:
        def __init__(self):
            self.parent_i = 0
            self.parent_j = 0
            self.f = float('inf')  # Total cost of the cell (g + h)
            self.g = float('inf')  # Cost from start to this cell
            self.h = 0             # Heuristic cost from this cell to destination

    def is_valid(self, row, col):
        return (row >= 0) and (row < self.ROW) and (col >= 0) and (col < self.COL)

    def is_unblocked(self, row, col):
        return self.grid[row][col] == 0

    def is_destination(self, row, col):
        return row == self.dest[0] and col == self.dest[1]

    def calculate_h_value(self, row, col):
        return ((row - self.dest[0]) ** 2 + (col - self.dest[1]) ** 2) ** 0.5
    
    def trace_path(self):
        path = []
        row = self.dest[0]
        col = self.dest[1]

        while not (self.cell_details[row][col].parent_i == row and self.cell_details[row][col].parent_j == col):
            path.append((row, col))
            temp_row = self.cell_details[row][col].parent_i
            temp_col = self.cell_details[row][col].parent_j
            row = temp_row
            col = temp_col

        path.append((row, col))
        path.reverse()
        return path

    def a_star_search(self):
        if not self.is_valid(self.src[0], self.src[1]) or not self.is_valid(self.dest[0], self.dest[1]):
            print("Source or destination is invalid")
            return

        if not self.is_unblocked(self.src[0], self.src[1]) or not self.is_unblocked(self.dest[0], self.dest[1]):
            print("Source or the destination is blocked")
            return

        if self.is_destination(self.src[0], self.src[1]):
            print("We are already at the destination")
            return


        closed_list = [[False for _ in range(self.COL)] for _ in range(self.ROW)]

        i, j = self.src
        self.cell_details[i][j].f = 0
        self.cell_details[i][j].g = 0
        self.cell_details[i][j].h = 0
        self.cell_details[i][j].parent_i = i
        self.cell_details[i][j].parent_j = j

        open_list = []
        heapq.heappush(open_list, (0.0, i, j))
        found_dest = False

        while len(open_list) > 0:
            p = heapq.heappop(open_list)
            i, j = p[1], p[2]
            closed_list[i][j] = True
                        #↓,     ^,      >,      <,  Diagonal>↓,     Diagonal>^,     Diagonal<↓,     Diagonal<^      
            directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
            for dir in directions:
                new_i, new_j = i + dir[0], j + dir[1]

                if self.is_valid(new_i, new_j) and self.is_unblocked(new_i, new_j) and not closed_list[new_i][new_j]:
                    if self.is_destination(new_i, new_j):
                        self.cell_details[new_i][new_j].parent_i = i
                        self.cell_details[new_i][new_j].parent_j = j
                        print("The destination cell is found")
                        self.trace_path()
                        found_dest = True
                        return
                    else:
                        g_new = self.cell_details[i][j].g + 1.0
                        h_new = self.calculate_h_value(new_i, new_j)
                        f_new = g_new + h_new

                        if self.cell_details[new_i][new_j].f == float('inf') or self.cell_details[new_i][new_j].f > f_new:
                            heapq.heappush(open_list, (f_new, new_i, new_j))
                            self.cell_details[new_i][new_j].f = f_new
                            self.cell_details[new_i][new_j].g = g_new
                            self.cell_details[new_i][new_j].h = h_new
                            self.cell_details[new_i][new_j].parent_i = i
                            self.cell_details[new_i][new_j].parent_j = j

        if not found_dest:
            print("Failed to find the destination cell")
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

class Simulator(object):
    def main(self, screen, robot):
        clock = pygame.time.Clock()
        mx = WIDTH / 2
        my = HEIGHT * 0.85

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    return
                if event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = pygame.mouse.get_pos()

            screen.fill(white)  # Fill the screen with white at the beginning of each iteration

            # Draw the robot
            robot.set(mx, my)
            robot.draw(screen)
            robot.data_analyzing(screen)
            # Render and display the text
            font = pygame.font.Font(None, 36)
            text = font.render(f'mx: {mx}, my: {my}', True, black)
            screen.blit(text, (0, 0))

            pygame.display.flip()
            clock.tick(20)

if __name__ == '__main__':
    pygame.init()
    pygame.display.set_caption('Leg Simulation')
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    robot = Robot()
    Simulator().main(screen, robot)
    pygame.quit()
    
    # Plot the collected data using scatter plots
    fig, ax1 = plt.subplots()

    # Scatter x and y data
    ax1.scatter(robot.x_data, robot.y_data, color='b', label='Distance')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y', color='b')
    
    ax2 = ax1.twinx()
    ax2.scatter(robot.x_data, robot.q1_data, color='r', label='q1 Angle')
    ax2.scatter(robot.x_data, robot.q2_data, color='y', label='q2 Angle')
    ax2.set_ylabel('q1 and q2 in Degrees', color='black')

    vertical_lines_indices = [0, 5, 10, 15, 20, 25]
    vertical_lines_x = [robot.x_data[i] for i in vertical_lines_indices]
    for x in vertical_lines_x:
        ax2.axvline(x=x, color='gray', linestyle='--', linewidth=.5)
        
        
    ax2.axhline(y=-88, color='r', linestyle='--', linewidth=1, label='Upper limit of q1')
    ax2.axhline(y=57, color='yellow', linestyle='--', linewidth=1, label='Lower limit of q2')


    plt.legend()
    plt.title("Robot Movement distance~q1,q2")
    plt.show()
