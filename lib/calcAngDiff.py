import numpy as np


def calcAngDiff(R_des, R_curr):
    """
    Helper function for the End Effector Orientation Task. Computes the axis of rotation 
    from the current orientation to the target orientation

    This data can also be interpreted as an end effector velocity which will
    bring the end effector closer to the target orientation.

    INPUTS:
    R_des - 3x3 numpy array representing the desired orientation from
    end effector to world
    R_curr - 3x3 numpy array representing the "current" end effector orientation

    OUTPUTS:
    omega - 0x3 a 3-element numpy array containing the axis of the rotation from
    the current frame to the end effector frame. The magnitude of this vector
    must be sin(angle), where angle is the angle of rotation around this axis
    """
    omega = np.zeros(3)
    ## STUDENT CODE STARTS HERE
    R_diff = R_curr.T @ R_des # compute the rotation difference matrix that rotates current rotation to R_des
    # trace = np.trace(R_diff) #checks sum of diagonals

    # if np.isclose(trace, 3.0): # if it is close to 3 (aka identity -> zero rotation)
    #     return np.zeros(3) # return 0

    skew = (R_diff - R_diff.T)/2 # compute the skew matrix of R_diff
    omega_part = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
    omega = R_curr @ omega_part
    
    return omega