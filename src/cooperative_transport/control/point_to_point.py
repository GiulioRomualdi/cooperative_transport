import numpy as np
import cooperative_transport.utils as utils

class PointToPoint:
    """Point to point control of unicycle-type robot.

    Attributes:
    xg (float): goal x-coordinate
    yg (float): goal y-coordinate
    max_forward_velocity (float): forward velocity saturation value
    max_angular_velocity (float): angular velocity saturation value
    
    """
    def __init__(self, max_forward_velocity, max_angular_velocity):
        """Initilize the object.

        Arguments:
        max_forward_velocity (float): forward velocity saturation value
        max_angular_velocity (float): angular velocity saturation value
        robot_state_0 (float[]): robot initial state
        """
        self.xg = 0
        self.yg = 0
        self.max_forward_velocity = max_forward_velocity
        self.max_angular_velocity = max_angular_velocity
    
    
    def goal_point(self, point):
        """Set goal point.

        Arguments:
        point (float[]): goal point coordinates
        """
        self.xg = point[0]
        self.yg = point[1]

    def control_law(self, robot_state, robot_velocity):
        """Return controller output

        Arguments:
        robot_state (float[]): goal point coordinates
        robot_velocity (float): forward robot velocity
        """
        xr = robot_state[0]
        yr = robot_state[1]
        theta = robot_state[2]
        vr = robot_velocity

        k = 1

        rho_rg = np.sqrt((xr - self.xg) ** 2 + (yr - self.yg) ** 2)
        phi_rg = np.arctan2((self.yg - yr), (self.xg - xr))
        phi_rg_dot = -(vr * np.sin(theta - phi_rg)) / rho_rg

        if rho_rg > 0.01:
            forward_velocity = rho_rg
            angular_velocity = -k * utils.angle_normalization(theta - phi_rg) + phi_rg_dot

            saturated_forward_velocity = utils.saturation(rho_rg, self.max_forward_velocity)
            saturated_angular_velocity = utils.saturation(angular_velocity, self.max_angular_velocity)
                
            return False, saturated_forward_velocity, saturated_angular_velocity

        return True, 0, 0
