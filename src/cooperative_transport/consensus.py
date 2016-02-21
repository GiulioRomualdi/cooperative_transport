def consensus(state, neighbours_state, reference):
    """Return consensus controller output for a SISO system.
    
    Arguments:
    state (float): robot angular state
    neighbors_state (float[]): neighbors angular state
    reference (float): desired angular state
    """
    neighbours_state.append(reference)
    output = sum([neighbour_state - state for neighbour_state in neighbours_state])
    return output
