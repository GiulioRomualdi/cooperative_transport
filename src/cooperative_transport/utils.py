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
