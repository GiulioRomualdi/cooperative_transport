#!/usr/bin/env python

PKG = 'cooperative_transport'
NAME = 'box_geometry_test'

import unittest
import rosunit
import numpy as np
from cooperative_transport.box import BoxGeometry

class TestBoxGeometry(unittest.TestCase):
    """TestCase class for cooperative_transport.box.BoxGeometry."""

    def test_vertex_rl(self):
        """Test the evaluation of the rear left vertex."""

        # RL = 0
        vertex_index = 0

        length = 2
        delta_x = float(length)/2
        width = 1
        delta_y = float(width)/2
        center = [5,5]
        theta = np.pi/3

        box_geometry = BoxGeometry(length, width, center, theta)
        
        vertex = box_geometry.vertex(vertex_index)
        
        expected_vertex_x = center[0] - delta_x * np.cos(theta) + delta_y * np.sin(theta)
        expected_vertex_y = center[1] - delta_x * np.sin(theta) - delta_y * np.cos(theta)

        outcome = (abs(expected_vertex_x - vertex[0]) < 1e-6) and\
                  (abs(expected_vertex_y - vertex[1]) < 1e-6)

        self.assertTrue(outcome)

    def test_vertex_rr(self):
        """Test the evaluation of the rear right vertex."""

        # RR = 1
        vertex_index = 1

        length = 2
        delta_x = float(length)/2
        width = 1
        delta_y = float(width)/2
        center = [5,5]
        theta = np.pi/3

        box_geometry = BoxGeometry(length, width, center, theta)
        
        vertex = box_geometry.vertex(vertex_index)
        
        expected_vertex_x = center[0] + delta_x * np.cos(theta) + delta_y * np.sin(theta)
        expected_vertex_y = center[1] + delta_x * np.sin(theta) - delta_y * np.cos(theta)

        outcome = (abs(expected_vertex_x - vertex[0]) < 1e-6) and\
                  (abs(expected_vertex_y - vertex[1]) < 1e-6)

        self.assertTrue(outcome)

    def test_vertex_fr(self):
        """Test the evaluation of the front right vertex."""

        # FR = 2
        vertex_index = 2

        length = 2
        delta_x = float(length)/2
        width = 1
        delta_y = float(width)/2
        center = [5,5]
        theta = np.pi/3

        box_geometry = BoxGeometry(length, width, center, theta)

        vertex = box_geometry.vertex(vertex_index)
        
        expected_vertex_x = center[0] + delta_x * np.cos(theta) - delta_y * np.sin(theta)
        expected_vertex_y = center[1] + delta_x * np.sin(theta) + delta_y * np.cos(theta)

        outcome = (abs(expected_vertex_x - vertex[0]) < 1e-6) and\
                  (abs(expected_vertex_y - vertex[1]) < 1e-6)

        self.assertTrue(outcome)

    def test_vertex_fl(self):
        """Test the evaluation of the front left vertex."""

        # FL = 3
        vertex_index = 3

        length = 3
        delta_x = float(length)/2
        width = 1
        delta_y = float(width)/2
        center = [5,5]
        theta = np.pi/3

        box_geometry = BoxGeometry(length, width, center, theta)
        
        vertex = box_geometry.vertex(vertex_index)
        
        expected_vertex_x = center[0] - delta_x * np.cos(theta) - delta_y * np.sin(theta)
        expected_vertex_y = center[1] - delta_x * np.sin(theta) + delta_y * np.cos(theta)

        outcome = (abs(expected_vertex_x - vertex[0]) < 1e-6) and\
                  (abs(expected_vertex_y - vertex[1]) < 1e-6)

        self.assertTrue(outcome)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'box_geometry_test', TestBoxGeometry)

    
