# Robotics Assessment One

Student Name: Lewis Taylor
Student ID: C1009129

Video Link (5:26) - https://youtu.be/mOQSVMRQYEw

# Tasks undertaken
- All basic requirements
- Find the quickest route out of the maze
- Map the maze
- Return back to home

# Structure of the code
The program consists of four classes:

- Point: This class represents a 250mm x 250mm square of the maze, and holds the information for the walls
- Maze: This is a class containing the Points needed to represent the whole maze
- Solver: This class contains the code to take in a maze object and return the fastest path to the exit using a breadth first search
- Robot: This class is used to control the robot, such as moving ad turning, and holding the stack of points and directions needed to get to the current point, forming a depth first search traversal method.

# Potential Improvements

- Mapping the maze takes a lot of time and could be sped up, currently when scanning a wall only the current point is updated, but the other side of the wall can also be interpretted. 
- When looking down a corridor it's possible to determine how many points there are until the next point with a wall, all those points could be updated from the original point where the scan took place.
- Unmapped points could be interpretted when there is only one possible state for them, such as if every point on the maze but one has been mapped, the robot should be able to interpret the state of the last point without physically visiting it. This however, would add more assumptions about the maze into the code, such as it definitely being an 8x8 maze of 250mm x 250mm squares.
- The code could be rewritten to not make assumptions of the maze, instead, scaling the size of the maze in the robot's memory to what has been determined by the sensors.
