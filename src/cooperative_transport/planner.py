from ompl import base as ob
from ompl import geometric as og
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class RectangularObstacle:
    """Implement obstacle in C-Space as rectangular shape.

    Attributes:
    length (float): box length in meters
    width (float): box width in meters
    xc (float): barycenter x-coordinate
    yc (float): barycenter y-coordinate
    theta (float): rectangle asset
    """

    def __init__(self, length, width, xc, yc, theta, robot_radius):
        """Initialize the object.
        
        Arguments:
        length (float): box length in meters
        width (float): box width in meters 
        xc (float): barycenter x-coordinate
        yc (float): barycenter y-coordinate
        theta (float): rectangle attitude
        robot_radius (float): robot radius in meters
        """
        # Define an obstacle region in C-Space
        tolerance = 0.05
        self.length = length + 2*robot_radius + 2*tolerance
        self.width = width + 2*robot_radius + 2*tolerance
        self.xc = xc
        self.yc = yc
        self.theta = theta

    def is_valid (self, x, y):
        """Return True if the point is outside the obstacle region.
        
        Arguments:
        x (float): x-coordinate
        y (float): y-coordinate
        """
        value = 2 * (-1 + abs(((self.width * x - self.width * self.xc -\
                self.length * y + self.length * self.yc) * np.cos(self.theta) +\
                (self.length * x - self.length * self.xc + self.width * y -\
                self.width * self.yc) * np.sin(self.theta)) / (self.length * self.width)) +\
                abs(((self.width * x - self.width * self.xc + self.length * y -\
                self.length * self.yc) * np.cos(self.theta) + (- self.length * x +\
                self.length * self.xc + self.width * y - self.width * self.yc) *\
                np.sin(self.theta)) / (self.length * self.width)))

        return value > 0

    def change_coordinates(self, x, y, theta):
        """Change the obstacle coordinates.

        Arguments:
        x (float): barycenter x-coordinate
        y (float): barycenter y-coordinate
        theta (float): rectangle attitude  
        """
        self.xc = x
        self.yc = y
        self.theta = theta 

class CircularObstacle:
    """Implement obstacle in C-Space as circular shape.
                                                                                
    Attributes:                                                                 
    radius (float): circle radius in meters                                       
    xc (float): barycenter x-coordinate
    yc (float): barycenter y-coordinate
    """
    def __init__(self, radius, xc, yc, robot_radius):
        """Initialize the object.
        
        Arguments:
        radius (float): circle radius in meters                                       
        xc (float): barycenter x-coordinate
        yc (float): barycenter y-coordinate
        robot_radius (float): robot radius in meters
        """

        # Define obstacle ragion in C-Space
        tolerance = 0.05
        self.radius = radius + robot_radius + tolerance
        self.xc = xc
        self.yc = yc
        
    def is_valid (self, x, y):
        """ Return True if the coordinate is extern to obstacle region.
        
        Arguments:
        x (float): x-coordinate
        y (float): y-coordinate
        """
        value = float(np.sqrt((x - self.xc)**2 + (y - self.yc)**2) - self.radius)
        return value > 0

    def change_coordinates(self, x, y):
        """Change the obstacle coordinates.

        Arguments:
        x (float): barycenter x-coordinate
        y (float): barycenter y-coordinate
        """
        self.xc = x
        self.yc = y
                
class Planner:
    """Implement an RRT* planner."""

    def __init__(self, lower_bound, upper_bound):
        """Initialize the object."""
        # Create an R2 state space
        self.space = ob.RealVectorStateSpace(2)

        # Set lower and upper bounds
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(lower_bound)
        bounds.setHigh(upper_bound)
        self.space.setBounds(bounds)
    
        # Create a new space information
        self.si = ob.SpaceInformation(self.space)

        # Set the state validity checker
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
        self.si.setup()

        # Set the problem definition
        self.pdef = ob.ProblemDefinition(self.si)

        # Setup an optimizing planner with RRT*
        self.optimizingPlanner = og.RRTstar(self.si)

        # Empty obstacles list
        self.obstacles = []

    def add_obstacle(self, obstacle):
        """Add an obstacle region in C-Space.

        Arguments:
        obstacle (Obstacle*): obstacle region
        """
        self.obstacles.append(obstacle)
        
    def is_state_valid(self, state):
        """Return True if the state is within an obstacle region.

        Arguments:
        state (): point coordinates
        """
        x = state[0]
        y = state[1]
        
        for obstacle in self.obstacles:
            if not obstacle.is_valid(x, y):
                return False

        return True 

    def plan(self, start, goal):
        """ Return the optimized path.

        Argument:
        start (float[]): the starting point
        goal (float[]): the goal point
        """        
        # Set start point
        start_state = ob.State(self.space)
        start_state()[0] = start[0]
        start_state()[1] = start[1]
        
        # Set goal point
        goal_state = ob.State(self.space)
        goal_state()[0] = goal[0]
        goal_state()[1] = goal[1]
   
        # Set start and goal points in the problem definition
        self.pdef.setStartAndGoalStates(start_state, goal_state)
        
        # Set the updated problem definition in the RRT* planner
        self.optimizingPlanner.setProblemDefinition(self.pdef)
        self.optimizingPlanner.setup()

        # Try to find the path
        solved = self.optimizingPlanner.solve(1.0)
    
        if solved:
            path = self.pdef.getSolutionPath()
            states = path.getStates()
            vector = [[state[0], state[1]] for state in states]
            print vector

            return True, vector

        return False,
