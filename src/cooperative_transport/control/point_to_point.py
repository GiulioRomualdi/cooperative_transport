import numpy as np
import cooperative_transport.utils as utils

class PointToPoint:
    """Point to point control of unicycle-type robot.

    Attributes:
        xg (float): goal x-coordinate
        yg (float): goal y-coordinate
        max_forward_velocity (float): forward velocity saturation value
        max_angular_velocity (float): angular velocity saturation value
        robot_radius (float): robot radius in meters (requested in obstacle avoidance mode)

    """
    def __init__(self, max_forward_velocity, max_angular_velocity, robot_radius = []):
        """Initilize the object.

        Arguments:
            max_forward_velocity (float): forward velocity saturation value
            max_angular_velocity (float): angular velocity saturation value
            robot_state_0 (float[]): robot initial state
            robot_radius (float): robot radius in meters (requested in obstacle avoidance mode)
        """
        self.xg = 0
        self.yg = 0
        self.max_forward_velocity = max_forward_velocity
        self.max_angular_velocity = max_angular_velocity
        self.robot_radius = robot_radius
    
    def goal_point(self, point):
        """Set goal point.

        Arguments:
            point (float[]): goal point coordinates
        """
        self.xg = point[0]
        self.yg = point[1]

    def control_law(self, robot_state, robot_velocity, obstacle_avoidance = False, robots_state = [], robots_velocity = []):
        """Return controller output

        Arguments:
            robot_state (float[]): robot state (x, y, theta)
            robot_velocity (float): forward robot velocity
            obstacle_avoidance (boolean): enable obstacle avoidance mode
            robots_state (float[[]]): other robots state necessary in obstacle avoidance mode
            robots_velocity (float[]): other forward robots velocity necessary in obstacle avoidance mode

        """
        xr = robot_state[0]
        yr = robot_state[1]
        theta = robot_state[2]
        vr = robot_velocity

        k = 1

        rho_rg = np.sqrt((xr - self.xg) ** 2 + (yr - self.yg) ** 2)
        phi_rg = np.arctan2((self.yg - yr), (self.xg - xr))
        phi_rg_dot = -(vr * np.sin(theta - phi_rg)) / rho_rg

        # Obstacle avoidance mode
        if obstacle_avoidance:
            rho_ros = [np.sqrt((robot_state[0] - xr) ** 2 + (robot_state[1] - yr) ** 2) for robot_state in robots_state]

            # Nearest obstacle
            index_min = np.argmin(rho_ros)
            rho_ro = rho_ros[index_min]
            
            x_ob = robots_state[index_min][0]
            y_ob =  robots_state[index_min][1]
            theta_ob = robots_state[index_min][2]
            v_ob = robots_velocity[index_min]

            phi_ro = np.arctan2((yr - y_ob), (xr - x_ob))
            phi_ro_dot = (-(v_ob * np.sin(theta_ob - phi_ro)) +\
                          (vr * np.sin(theta - phi_ro))) / rho_ro

            if rho_ro < 2 * self.robot_radius + 0.2:
                delta = utils.angle_normalization(theta - phi_ro)+ phi_ro_dot

                forward_velocity = 0
                if abs(delta) < np.pi / 18:
                    forward_velocity = 0.1

                angular_velocity = -k * delta + phi_ro_dot
                
                saturated_forward_velocity = utils.saturation(forward_velocity, self.max_forward_velocity)
                saturated_angular_velocity = utils.saturation(angular_velocity, self.max_angular_velocity)
                
                return False, saturated_forward_velocity, saturated_angular_velocity

        if rho_rg > 0.01:
            forward_velocity = rho_rg
            if obstacle_avoidance:
                forward_velocity = 0.2
                if rho_rg < 0.2:
                    forward_velocity = rho_rg
            angular_velocity = -k * utils.angle_normalization(theta - phi_rg) + phi_rg_dot

            saturated_forward_velocity = utils.saturation(forward_velocity, self.max_forward_velocity)
            saturated_angular_velocity = utils.saturation(angular_velocity, self.max_angular_velocity)
        
            return False, saturated_forward_velocity, saturated_angular_velocity

        return True, 0, 0
