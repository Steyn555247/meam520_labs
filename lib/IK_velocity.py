import numpy as np 
from lib.calcJacobian import calcJacobian



def IK_velocity(q_in, v_in, omega_in):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param v_in: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega_in: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 1 x 7 vector corresponding to the joint velocities. If v_in and omega_in
         are infeasible, then dq should minimize the least squares error. If v_in
         and omega_in have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """
    dq = np.zeros((1, 7))

    v_in = v_in.reshape((3,1))
    omega_in = omega_in.reshape((3,1))
    xi = np.vstack((omega_in, v_in)) # to store xi values from v_in and omega_in

    ## STUDENT CODE GOES HERE
    J=calcJacobian(q_in)
    def solution_exists(J, xi, tol=1e-6):
        rank_J = np.linalg.matrix_rank(J, tol=tol) #compute rank of Jacobian to test J|xi to later

        augmented_matrix = np.hstack((J, xi)) #construct the J, Xi matrick
        rank_augmented = np.linalg.matrix_rank (augmented_matrix, tol=tol) # rank the matrix of J|xi

        return rank_J == rank_augmented # check if the ranks are equal
    
    if solution_exists(J, xi):
        print("solution exists, computing exact inverse")
        J_inverse = np.linalg.pinv(J)
        dq = J_inverse @ xi

    
    else: 
         print("no exact solution exists, finding least square error")
         dq, residuals, rank, singular_values = np.linalg.lstsq(J, xi, rcond=None)
         print(f"Least squares solution: {dq}")
         print(f"Residual error: {residuals}")
         print(f"Rank of J: {rank}")
         print(f"Singular values: {singular_values}")
       
   
    return dq
