#region VEXcode Generated Robot Configuration
import math
import random
from vexcode_vr import *
from collections import deque

# Brain should be defined by default
brain=Brain()

drivetrain = Drivetrain("drivetrain", 0)
pen = Pen("pen", 8)
pen.set_pen_width(THIN)
left_bumper = Bumper("leftBumper", 2)
right_bumper = Bumper("rightBumper", 3)
front_eye = EyeSensor("frontEye", 4)
down_eye = EyeSensor("downEye", 5)
front_distance = Distance("frontdistance", 6)
distance = front_distance
magnet = Electromagnet("magnet", 7)
location = Location("location", 9)

#endregion VEXcode Generated Robot Configuration
# ------------------------------------------
# 
# 	Project:      VEXcode Project
#	Author:       VEX
#	Created:
#	Description:  VEXcode VR Python Project
# 
# ------------------------------------------

# Add project code in "main"
def main():
    brain.clear()
    maze = Maze(8)
    robot = Robot(maze)
    robot.explore_maze()

# Maze is to hold the point objects in a structure similar to the maze's simulated state and extra information
class Maze:
    def __init__(self, size):
        self.size = size
        self.grid = [[Point(x, y) for y in range(size)] for x in range(size)]
        self.finish_position = None
        self.start_position = self.grid[4][0]
        self.fastest_path = None
        self.fastest_path_points = None

    # Takes the maze object's grid and prints out an ascii representation with the fastest path also marked
    def print_maze(self):
        space = ' '
        wall = '#'
        start = 'S'
        finish = 'F'
        fastest_path = '*'
        
        ascii_size = self.size * 2 + 1 # scales the size to allow for walls
        
        ascii = [[wall for _ in range(ascii_size)] for _ in range(ascii_size)]
        
        for row in self.grid:
            for cell in row:
                x = cell.x * 2 + 1
                y = (self.size - 1 - cell.y) * 2 + 1 # Invert y to match grid coordinates
                
                special = None
                if cell.x == self.start_position.x and cell.y == self.start_position.y:
                    special = "S"
                elif cell.x == self.finish_position.x and cell.y == self.finish_position.y:
                    special = "F"
                else:
                    found = False
                    for point in self.fastest_path_points:
                        if cell.x == point.x and cell.y == point.y:
                            found = True

                    if found:
                        special = fastest_path
                    else:
                        special = space
                

                ascii[y][x] = special
                    
                if "NORTH" in cell.walls: # Wall present
                    ascii[y-1][x] = wall
                else: # No wall present
                    ascii[y-1][x] = space
                    
                if "EAST" in cell.walls: # Wall present
                    ascii[y][x+1] = wall
                else: # No wall present
                    ascii[y][x+1] = space
                    
                if "SOUTH" in cell.walls: # Wall present
                    ascii[y+1][x] = wall
                else: # No wall present
                    ascii[y+1][x] = space
                    
                if "WEST" in cell.walls: # Wall present
                    ascii[y][x-1] = wall
                else: # No wall present
                    ascii[y][x-1] = space

        for row in ascii:
            output = ""
            for char in row:
                output += char
            brain.print(output)
            brain.new_line()

    # Calls the point's explore function and prints out the total percent of cells which have been mapped
    def explore_point(self, point):
        point.explore()

        total_explored = 0

        for i in self.grid:
            for ii in i:
                if ii.explored:
                    total_explored += 1

        total_size = self.size * self.size

        percent_mapped = total_explored / total_size * 100
        brain.print(f"Maze has been {percent_mapped}% mapped")
        brain.new_line()

    # Takes current point and the direction and returns the point in the grid in that direction
    def get_relative_position(self, current_point, direction):
        x, y = current_point.x, current_point.y

        if direction == "NORTH":
            y += 1
        elif direction == "SOUTH":
            y -= 1
        elif direction == "EAST":
            x += 1
        elif direction == "WEST":
            x -= 1
        else:
            brain.print(f"Invalid direction: {direction}")

        if 0 <= x < len(self.grid) and 0 <= y < len(self.grid[0]):
            return self.grid[x][y]
        else:
            return None

# Handles the "thinking" of the robot and the movement
class Robot:
    def __init__(self, maze):
        self.maze = maze
        self.current_point = maze.grid[4][0]
        self.stack = []

    # Aims to map the maze in a depth first search manner, return to the starting point, 
    # figure out the fastest path in a breadth first search manner to the end and then traverse that path

    # DFS was chosen as the mapping algorithm as it will have as little traversal time as possible as the robot
    # exhausts the path in front of it before backtracking

    # BFS was chosen to calculate the fastest path due to is guarenteeing the shortest path from the origin to the destination,
    # this is because all paths it's exploring are the same length, so the first one to reach the destination must the shortest path.
    # While this will result in the shortest path, if you factor in turns it's possible that another path with less turns would take
    # less time once you factor in time spent turning rather than moving.
    def explore_maze(self):
        brain.print("Starting to explore maze")
        brain.new_line()

        self.maze.explore_point(self.current_point)

        # The stack represents the paths explored and the direction taken
        # when an unexplored adjacent point is found it is added to the stack and the robot moves to that position
        # if a dead end is found then pop the last entry 
        self.stack = [(self.current_point, None)]

        while self.stack:
            # for debugging
            # for index, (point, direction) in enumerate(self.stack):
            #    brain.print(f"Stack[{index}]: Point({point.x}, {point.y}), Direction: {direction}")
            #    brain.new_line()
            
            if down_eye.detect(RED):
                self.maze.finish_position = self.current_point
                brain.print(f"Found finish point at x:{self.maze.finish_position.x} y:{self.maze.finish_position.y}")
                brain.new_line()

            self.current_point, direction = self.stack[-1]
            adjacent_point, next_direction = self.get_unexplored_adjacent(self.current_point)

            if adjacent_point: # There are unexplored adjacent cells to go to
                self.move(adjacent_point, [next_direction])
                #brain.print(f"Moved {next_direction} to x:{adjacent_point.x} y:{adjacent_point.y}")
                #brain.new_line()
                self.current_point = adjacent_point
                self.stack.append((self.current_point, next_direction))
                self.maze.explore_point(self.current_point)
            else: # Either a dead end or  break from the while loop as the entire maze is mapped
                previous_point, previous_direction = self.stack.pop()
                if previous_point: # if there is a previous point on the stack, go to the previous point on the stack
                    if previous_direction != None:
                        opposite_previous_direction = self.get_opposite_direction(previous_direction)
                        self.move(previous_point, [opposite_previous_direction])
                        #brain.print(f"Backtracked {opposite_previous_direction} to x:{previous_point.x} y:{previous_point.y}")
                        #brain.new_line()
                        self.current_point = previous_point
                    else:
                        break
                else:
                    raise ValueError("Error on finding previous point")

        while len(self.stack) > 1: # Entire maze is mapped so retake each step on the stack back to the start
            previous_point, previous_direction = self.stack.pop()
            opposite_previous_direction = self.get_opposite_direction(previous_direction)
            self.move(previous_point, [opposite_previous_direction])
            #brain.print(f"Backtracked {opposite_previous_direction} to x:{previous_point.x} y:{previous_point.y}")
            #brain.new_line()
            self.current_point = previous_point

        brain.print("Returned to start")
        brain.new_line()

        solver = Solver(self.maze)

        path, points = solver.solve_shortest_path()
        self.maze.fastest_path = path
        self.maze.fastest_path_points = points

        self.maze.print_maze()

        pen.set_pen_color(RED)
        pen.move(DOWN)
        self.move(self.maze.finish_position, path)

        brain.print("Maze solver finished")
        brain.new_line()
       
    # Take in the current point and return the first direction in directions
    # which is unexplored (via print.explored), does not have a wall between the 
    # points, and is not going out of bounds (the open walls for the start and finish points)
    def get_unexplored_adjacent(self, point):
        directions = ["NORTH", "SOUTH", "EAST", "WEST"]
        for direction in directions:
            if direction in point.walls:
                continue
                
            adjacent = self.maze.get_relative_position(point, direction)
            if adjacent is not None:
                oob = adjacent.x < 0 or adjacent.x > 7 or adjacent.y < 0 or adjacent.y > 7
            else:
                oob = True
            if adjacent and not adjacent.explored and not oob:
                return adjacent, direction
        return None, None

    def get_opposite_direction(self, direction):
        if direction == "NORTH":
            return "SOUTH"
        elif direction == "EAST":
            return "WEST"
        elif direction == "SOUTH":
            return "NORTH"
        elif direction == "WEST":
            return "EAST"
        else:
            return f"error: direction unknown {direction}"

    # Takes in the target of the move and the list of directions needed to get there
    def move(self, point, directions):
        for direction in directions:
            #brain.print(f"Moving to x={point.x} y={point.y} facing {direction}")
            #brain.new_line()

            heading = None

            if direction == "NORTH":
                heading = 0
            elif direction == "EAST":
                heading = 90
            elif direction == "SOUTH":
                heading = 180
            elif direction == "WEST":
                heading = 270
            else:
                raise ValueError(f"Invalid direction - {direction}")

            drivetrain.turn_to_heading(heading, DEGREES)
            drivetrain.drive_for(FORWARD, 250, MM)

            # If any bumper is pressed then throw an error as something has gone wrong
            if left_bumper.pressed() or right_bumper.pressed():
                raise ValueError("Error: collision with wall detected")

            self.current_point = self.maze.get_relative_position(point, direction)

# This class handles solving the fastest path through the maze using breadth first search
class Solver:
    def __init__(self, maze):
        self.maze = maze

    # Solve the maze using Breadth First Search
    def solve_shortest_path(self):
        start = self.maze.start_position
        end = self.maze.finish_position

        if start is None or end is None:
            raise ValueError("Start or finish position is not set.")

        queue = deque([(start, [], [])])  # (current_point, path_taken)
        visited = set()

        while queue:
            current, path, points = queue.popleft()

            if current in visited:
                continue

            visited.add(current)

            # If we reach the final position, return the directions to get there and the points in that path
            if current == end:
                return path, points + [current]

            # Get unvisited adjacent points with their movement directions
            for neighbor, direction in self.get_adjacent_points(current):
                if neighbor not in visited:
                    queue.append((neighbor, path + [direction], points + [current]))  # Extend the path

        return None  # No path found

    # Returns all points in the directions from the point given in the directions array,
    # where there are no walls in between those points or the point is not out of bounds
    def get_adjacent_points(self, point):
        directions = ["NORTH", "SOUTH", "EAST", "WEST"]
        adjacents = []

        for direction in directions:
            if direction in point.walls:
                continue
                
            adjacent = self.maze.get_relative_position(point, direction)
            if adjacent is not None:
                oob = adjacent.x < 0 or adjacent.x > 7 or adjacent.y < 0 or adjacent.y > 7
            else:
                oob = True
            if adjacent and not oob:
                adjacents.append((adjacent, direction))

        return adjacents

# The Point class represents a 250mm x 250mm square of the maze and holds the position of that square
class Point:
    def __init__(self, x = None, y = None):
        self.x = x
        self.y = y
        self.explored = False
        self.walls = []
        self.known = []
        self.special = None

    # scans whether there are walls in each 4 directions by spinning around and using the front eye's get_distance method
    def explore(self):
        for i in range(4):
            currentRotation = drivetrain.heading(DEGREES)

            if(front_distance.get_distance(MM) < 200):
                if currentRotation == 0:
                    self.walls.append("NORTH")
                elif currentRotation == 90:
                    self.walls.append("EAST")
                elif currentRotation == 180:
                    self.walls.append("SOUTH")
                elif currentRotation == 270:
                    self.walls.append("WEST")
                else:
                    brain.print(f"Heading is not correct - {drivetrain.rotation(DEGREES)}")
                    brain.new_line()

            drivetrain.turn_for(RIGHT, 90, DEGREES)

        self.explored = True

    def __str__(self):
        return f"({self.x}, {self.y})"

    def __repr__(self):
        return f"Point(x={self.x}, y={self.y}, explored={self.explored})"

# ************************************
# Potential improvements

# Exploring points could be improved by only scanning in directions which aren't known about as when scanning a wall,
# we also know that there is a space on the other side of a wall, or when looking down a corridor we know that there are distance/250mm cells
# without walls in that direction
# ************************************

# VR threads — Do not delete
vr_thread(main)