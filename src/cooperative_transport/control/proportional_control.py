from cooperative_transport.utils import saturation
import numpy as np

def proportional_control(k_p, r, y, u_max):
    """Implement proportional control law.

    Arguments:
        k_p (float): Proportional gain
        r (float): reference signal
        y (float): system output signal
        u_max (float): maximum control effort
    """
    error = r - y

    u = k_p * error
    saturated_u = saturation(u ,u_max)
    return saturated_u
