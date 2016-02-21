from numpy import sign

def consensus(state, neighbours_state, reference, max_control):
    """Return consensus controller output for a SISO system.
    
    Arguments:
    state (float): robot angular state
    neighbors_state (float[]): neighbors angular state
    reference (float): desired angular state
    max_control (float): maximum allowed output
    """
    neighbours_state.append(reference)
    output = sum([neighbour_state - state for neighbour_state in neighbours_state])
    saturated_output = sign(output) * min(abs(output), max_control)
    return saturated_output
