#!/usr/bin/env python

PKG = 'cooperative_transport'
NAME = 'planner_test'

import unittest
import rosunit
import numpy as np
from cooperative_transport.planner import *

class TestPlanner(unittest.TestCase):
    """TestCase class for cooperative_transport.planner.Planner."""

    def test_end_point_close_to_goal(self):
        """Test if the last point in the planned path is almost close to the goal point."""
        planner = Planner(-5, 5)

        # obstacles
        robot_radius = 0.1696
        planner.add_obstacle(CircularObstacle(robot_radius, -1.0, 1.0 , robot_radius))
        planner.add_obstacle(CircularObstacle(robot_radius, -2.0, -2.0 , robot_radius))
        planner.add_obstacle(RectangularObstacle(1.0, 0.5, 0.0, 0.0, 0.0, robot_radius))

        # start and goal
        start_point = [3.0, 3.0]
        tolerance = 0.02
        goal_point = [-0.5 -robot_radius -tolerance, 0]

        # plan
        outcome, path = planner.plan(start_point, goal_point)

        self.assertTrue(np.allclose(goal_point, path[-1]),
                        'expected last point in path (' + `goal_point[0]` + ', ' + `goal_point[1]` + ') ' +
                        'but obtained (' + `path[-1][0]` + ', ' + `path[-1][1]` + ')')
    
if __name__ == '__main__':
    rosunit.unitrun(PKG, 'planner_test', TestPlanner)
