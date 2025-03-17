from XRPLib.defaults import *

from lib.XRPLib.pid import PID
from lib.XRPLib.timeout import Timeout

import time
import math

def sign(x):
    if x >= 0:
        return 1
    else:
        return -1


class MazeAgent:
    def __init__(self):
        self.maze = self.generate_maze(16)
        self.orientation = "east"
        self.location_y = 1
        self.location_x = 1
        self.add_wall(2, 1)

    def generate_maze(self, cells):
        """Generates an n x n array following the quadrant-based pattern logic."""
        n = cells*2 + 1
        array = np.zeros((n, n), dtype=int)
        center = cells  # Find the center of the grid

        for i in range(0,cells,2):
            for j in range(0,cells,2):
                value = (j+i)/2
                array[center + j + 1, center - i -1] = value
                array[center + j + 1, center + i + 1] = value
                array[center - j -1, center - i -1] = value
                array[center - j -1, center + i + 1] = value
        #outer wall
        for i in range(0,cells,2):
            array[0 , center - i -1] = 1
            array[0 , center + i +1] = 1
            array[n-1, center - i -1] = 1
            array[n-1, center + i +1] = 1
        for j in range(0,cells,2):
            array[center + j+1 , 0] = 1
            array[center + j+1 , n-1 ] = 1
            array[center - j-1, 0 ] = 1
            array[center - j-1, n-1 ] = 1

        return array

    def add_wall(self,y,x):
        self.maze[y][x] = 1

    def add_left_wall(self):
        if (self.orientation == "north"):
            self.add_wall(self.location_y, self.location_x-1)
        elif (self.orientation == "east"):
            self.add_wall(self.location_y-1, self.location_x)
        elif (self.orientation == "south"):
            self.add_wall(self.location_y, self.location_x+1)
        elif (self.orientation == "west"):
            self.add_wall(self.location_y+1, self.location_x)
    
    def add_right_wall(self):
        if (self.orientation == "north"):
            self.add_wall(self.location_y, self.location_x+1)
        elif (self.orientation == "east"):
            self.add_wall(self.location_y+1, self.location_x)
        elif (self.orientation == "south"):
            self.add_wall(self.location_y, self.location_x-1)
        elif (self.orientation == "west"):
            self.add_wall(self.location_y-1, self.location_x)
    
    def add_forward_wall(self):
        if (self.orientation == "north"):
            self.add_wall(self.location_y-1, self.location_x)
        elif (self.orientation == "east"):
            self.add_wall(self.location_y, self.location_x+1)
        elif (self.orientation == "south"):
            self.add_wall(self.location_y+1, self.location_x)
        elif (self.orientation == "west"):
            self.add_wall(self.location_y, self.location_x-1)

    def turn_orientation_right(self):
        if (self.orientation == "north"):
            self.orientation = "east"
        elif (self.orientation == "east"):
            self.orientation = "south"
        elif (self.orientation == "south"):
            self.orientation = "west"
        elif (self.orientation == "west"):
            self.orientation = "north"

    def turn_orientation_left(self):
        if (self.orientation == "north"):
            self.orientation = "west"
        elif (self.orientation == "east"):
            self.orientation = "north"
        elif (self.orientation == "south"):
            self.orientation = "east"
        elif (self.orientation == "west"):
            self.orientation = "south"

    def move_forward(self):
        if (self.orientation == "north"):
            self.location_y -= 2
        elif (self.orientation == "east"):
            self.location_x += 2
        elif (self.orientation == "south"):
            self.location_y += 2
        elif (self.orientation == "west"):
            self.location_x -= 2