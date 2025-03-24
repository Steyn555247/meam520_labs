import numpy as np
from lib.IK_velocity import IK_velocity
from lib.calcJacobian import calcJacobian

"""
Lab 3
"""

def IK_velocity_null(q_in, v_in, omega_in, b):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param v_in: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega_in: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :param b: 7 x 1 Secondary task joint velocity vector
    :return:
    dq + null - 1 x 7 vector corresponding to the joint velocities + secondary task null velocities
    """

    ## STUDENT CODE GOES HERE
    dq = np.zeros((1, 7))
    null = np.zeros((1, 7))
    b = b.reshape((7, 1))
    v_in = np.array(v_in)
    v_in = v_in.reshape((3,1))
    omega_in = np.array(omega_in)
    omega_in = omega_in.reshape((3,1))
    xi = np.vstack((v_in, omega_in)) # to store xi values from v_in and omega_in


    J=(calcJacobian(q_in))
    valid = ~np.isnan(xi[:, 0]) # filter out nan values
    if np.sum(valid) == 0:
        return dq
    
    J_valid = J[valid, :] #create subset of J with non NaN values
    xi_valid = xi[valid, :] #filter subset of xi with non NaN values since b is is the secondary task joint velocity vector

    dq = np.linalg.pinv(J_valid)@xi_valid

    # def null_space(J, tol=1e-12):
    #     U, s, Vh = np.linalg.svd(J) # find the Singular Value Decompisition of the Jacobian to find the null space of that specific jacobian

    #     null_mask = (s<tol) #idnetify the singular values that are basically zero

    #     null_space_basis = Vh[null_mask] #indicate which singular values are zero, those then are slected as basis from null space

    #     return null_space_basis.T
    
    J_pinv = np.linalg.pinv(J_valid) #compute the pseudoinverse from the Jacobian

    I=np.eye(7)

    N= I -J_pinv@J_valid

    null = N@b #compute null by the multiplying the secondary task with b

    return (dq + null).T

