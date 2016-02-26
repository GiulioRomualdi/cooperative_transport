#!/usr/bin/env python

PKG = 'cooperative_transport'
NAME = 'docking_test'

import unittest
import rosunit
import numpy as np
from cooperative_transport.box_info_services import find_docking_points

class TestDocking(unittest.TestCase):
    """TestCase class for cooperative_transport.box_info_services.find_docking_points function."""
    
    def test_no_angle_singular_goal_top(self):
        """Test the singular case with the goal on top and no angle.

        o := goal

                 o

        
          *=============*
          =             =
          =             =
          *=============*
        
        """
        box_length = 2
        box_width = 1
        box_current_pose = [5.0, 5.0, 0.0]
        box_goal_pose = [5.0, 10.0]

        docking = find_docking_points(box_current_pose, box_goal_pose, box_length, box_width)

        # Points
        points = []
        normals = []
        for item in docking:
            points.append(item['point'])
            normals.append(item['normal'])

        # Expected
        expected_points = [[5, 4.5], [4, 5], [6, 5]]
        expected_normals = [[0, 1], [1, 0], [-1, 0]]

        outcome = True
        for p in expected_points:
            if not p in points:
                outcome = False
                break
        if not np.allclose(normals, expected_normals):
            outcome = False

        self.assertTrue(outcome)

    def test_no_angle_normal_goal_top(self):
        """Test the normal case with the goal on top and no angle.

        o := goal

                     o

    
        
          *=============*
          =             =
          =             =
          *=============*
        
        """
        box_length = 3
        box_width = 1
        box_current_pose = [5.0, 5.0, 0.0]
        box_goal_pose = [7.0, 10.0]

        docking = find_docking_points(box_current_pose, box_goal_pose, box_length, box_width)

        # Points
        points = []
        normals = []
        for item in docking:
            points.append(item['point'])
            normals.append(item['normal'])

        # Expected
        expected_points = [[3.5, 5], [4.5, 4.5], [5.5, 4.5]]
        expected_normals = [[1, 0], [0, 1], [0, 1]]

        outcome = True
        for p in expected_points:
            if not p in points:
                outcome = False
                break
        if not np.allclose(normals, expected_normals):
            outcome = False

        self.assertTrue(outcome)

if __name__ == '__main__':
    rosunit.unitrun(PKG, 'box_geometry_test', TestDocking, coverage_packages=['cooperative_transport.box'])



