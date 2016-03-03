from cooperative_transport.utils import saturation 
import numpy as np

def correct_delta(error):
    if abs(error) > np.pi:
        error += -2 * np.pi * np.sign(error)

    return error

def consensus(state, neighbours_state, reference, max_control):
    """Return consensus controller output for a SISO system.
    
    Arguments:
        state (float): robot angular state
        neighbors_state (float[]): neighbors angular state
        reference (float): desired angular state
        max_control (float): maximum allowed output
    """
    neighbours_state.append(reference)
    output = sum([correct_delta(neighbour_state - state) for neighbour_state in neighbours_state])
    saturated_output = saturation(output, max_control)
    return saturated_output
