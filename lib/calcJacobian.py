import numpy as np
from math import pi, sin, cos
from lib.calculateFK import FK
# from lib.calculateFK import get_axis_of_rotation

def calcJacobian(q):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """
    fk = FK()     # Retrieve the DH parameters (both main and joint-specific) for the given configuration.
    endDHparameters, jointspcDHparameters = fk.get_dh_parameters(q)

    origins = [] #to store coordinates of joint center
    z_axes = [] # to store Z-axis (rotation axis) for each joint based in world frame

    T_current = np.identity(4) #start with base frame (identity)

    for i in range(7): #loop over all 7 joints
        z=T_current[:3,2].copy() # get z_axis from transformation matrix by taking third column
    
        mod_tranform = fk.dh_transform(*jointspcDHparameters[i])# compute modification that shifts frame to actual joint position
        T_joint = T_current@mod_tranform #combine current transform with offset to get joint center

        origin = T_joint[:3, 3].copy() #extract the postiion part of the transfomration matrix (XYZ)

        origins.append(origin) #save joint center
        z_axes.append(z) #save z axis

        #  compute main transfomr using the main end effector DH params
        main_tranform = fk.dh_transform(*endDHparameters[i])

        T_current = T_current @ main_tranform # update current transform by chaining main transform

    o_n = T_current[:3, 3].copy() #extract end effector position from main transform

    # initialize Jacobian matrix (empty)
    J = np.zeros((6, 7))

    #loop over each joint and compute the contrubtion to the full jacobian
    for i in range(7):

        #compute linear veloc by cross product of rotation axis and vector from o_n to o joint
        J_linear =np.cross (z_axes[i], (o_n - origins[i]))

        #angular veloc is just joint axis thus
        J_angular = z_axes[i]

        #take the J array and t linear in first three rows and angual in last three rows
        J[0:3, i] = J_linear
        J[3:6, i] = J_angular

    return J

    ## STUDENT CODE GOES HERE

    

if __name__ == '__main__':
    q= np.array([0, 0, 0, 0, 0, 0, 0])
    print(np.round(calcJacobian(q),3))