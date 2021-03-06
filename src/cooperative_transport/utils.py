from __future__ import division
import numpy as np

def quaternion_to_yaw(quaternion):
    """Extract euler yaw angle from a quaternion.

    Arguments:
        quaternion (Quaternion): quaternion 
    """
    q0 = quaternion.w
    q3 = quaternion.z
    yaw = np.arctan2(2 * (q0 * q3), 1 - 2 * q3 ** 2)
    return yaw

def angle_normalization(angle):
    """Keep angle between -pi and +pi

    Arguments:
        angle (float): angle
    """
    if angle <= -np.pi:
        normalized_angle = 2 * np.pi + angle
    elif angle > np.pi:
        normalized_angle = - 2 * np.pi + angle
    else:
        normalized_angle = angle
    return normalized_angle

def saturation(input_value, max_value):
    """Impose upper limit on an input signal.    

    Arguments:
        input_value (float): input signal
        max_value (float): saturation value
    """
    saturated_value = np.sign(input_value) * min(abs(input_value), max_value)
    return saturated_value

class Line:
    """Line between two points p0 and p1."""

    def __init__(self, p1 ,p2):
        self.a = p1[1] - p2[1]
        self.b = p2[0] - p1[0]
        self.c = p1[1] * (p1[0] - p2[0]) + p1[0] * (p2[1] - p1[1])

    def set_points(self, p1 ,p2):
        self.a = p1[1] - p2[1]
        self.b = p2[0] - p1[0]
        self.c = p1[1] * (p1[0] - p2[0]) + p1[0] * (p2[1] - p1[1])
    
    def distance(self, p):
        x = p[0]
        y = p[1]
        distance = abs(self.a * x + self.b *y + self.c) /\
                   np.sqrt(self.a ** 2 + self.b ** 2)
        
        return distance

    def evaluate(self, p):
        x = p[0]
        y = p[1]
        value = self.a * x + self.b *y + self.c
        
        return value

    def coefficients(self):
        return [self.a, self.b, self.c]

    def intersect(self, line):
        coeffs = line.coefficients()
        a2, b2, c2 = [coeffs[0], coeffs[1], coeffs[2]]
        determinant = (a2 * self.b - self.a * b2)
        sol_x = - (- b2 * self.c + self.b * c2) / determinant 
        sol_y = - (a2 * self.c - self.a * c2) / determinant

        return [sol_x, sol_y]

class Segment:
    """Segment between two points p0 and p1.

    p(t) = (1 - t) * p0 + t * p1 t in [0,1]
    """

    def __init__(self, p0, p1):
        self._p0 = np.array(p0)
        self._p1 = np.array(p1)

    @property
    def p0(self):
        return self._p0.tolist()
        
    @p0.setter
    def p0(self, p0):
        self._p0 = np.array(p0)

    @property
    def p1(self):
        return self._p1.tolist()

    @p1.setter
    def p1(self, p1):
        self._p1 = np.array(p1)

    def normal(self):
        """Return the normal direction pointing inward"""
        difference = self._p1 - self._p0
        length = np.linalg.norm(difference)
        normal = np.array([float(-difference[1]), float(difference[0])])/length

        return normal.tolist()

    def point(self, t):
        """Return the point p(t) = (1 - t) * p0 + t * p1.

        Arguments:
            t (float): parameter t in [0,1]
        """
        
        p = (1 - t) * self._p0 + t * self._p1

        return p.tolist()

    def length(self):
        """Return the length of the segment."""
        difference = self._p1 - self._p0
        return float(np.linalg.norm(difference))

    def __ge__(self, other):
        if isinstance(other, Segment):
            return self.length() >= other.length()
        else:
            return NotImplemented        
