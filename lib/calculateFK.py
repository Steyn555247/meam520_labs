#for this overal assignment I worked together with Matt Rabin. Together we constructed the frames and thought about the assignment and discussed how we were tackling the project
#all code written in this assignment is however my full work. We were just working collaborating on solving

import numpy as np
from math import pi, sin, cos

class FK():
    # def __init__(self):
    #     # DH Parameters: (theta, d, a, alpha) following Craig's standard DH convention.
    #     # These values come from the lab handout.
    #     #format rows as [a, alpha, d, theta]
        

    def dh_transform( self, a, alpha, d, theta): # Function for the standard DH transformation matrix so that it can be called easily
        """Return the standard DH transformation matrix given parameters."""
        return np.array([
            [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
            [0,           sin(alpha),            cos(alpha),            d],
            [0,           0,                     0,                     1]
        ])

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your Lab 1 code starts here
        # dh paramaters (a, alpha, d, theta)
        endDHparameters = np.array([
            [0,      -pi/2,   0.333,     q[0]],          # Joint 1
            [0,      pi/2,     0,    q[1]],       # Joint 2
            [0.0825, pi/2,   0.3160,    q[2]],        # Joint 3
            [-0.08250, -pi/2, 0,  q[3]],        # Joint 4
            [0,      pi/2,  0.384, q[4]],        # Joint 5
            [0.088,   pi/2,     0.0,     q[5]],        # Joint 6
            [0,      0.0,     0.21,   q[6]-(pi/4)]       # Joint 7
        ])

        jointspcDHparameters=np.array([ #specific parameters to adjust for distance between joint and frame
            [0, 0, 0.141, q[0]], 
            [0, 0, 0, q[1]],
            [0, 0, 0.195, q[2]],
            [0, 0, 0, q[3]],
            [0, 0, 0.125, q[4]],
            [0, 0, -0.015, q[5]],
            [0, 0, 0.051, q[6]]
        ])
       
        jointPositions = np.zeros((8, 3)) # start 8x3 zero matrix to store joints and end effector positions
        T0e = np.identity(4) # identity matrix start as base frame

        for i in range(7): #for loop through the 7 joints
            modtransform= self.dh_transform(*jointspcDHparameters[i]) #modification the transform of the additional DH parameters to adjust the frame to the actual joint center and standard transform Function
            maintransform = self.dh_transform(*endDHparameters[i]) #compute main transform for joint using primary DH param and standard transform function
            Tmod = T0e @ modtransform #multiply current T0e with modification transform to get current joint's center
            jointPositions[i] = Tmod[:3, 3].tolist() #add the Tmod value of the joints to the 3x3 matrix
            T0e = T0e @ maintransform # set T0e to the main transformation

            jointPositions[7] = T0e[:3, 3].tolist() #add transformation to the 3x3 matrix

        


        # Your code ends here

        return jointPositions, T0e

        # feel free to define additional helper methods to modularize your solution for lab 1

    def get_dh_parameters(self, q):
        """
        New function to directly return the DH parameters for the given configuration q.
        This function allows other parts of the code (e.g., Jacobian computation) to use the same DH parameters
        without having to duplicate the definitions here.
        
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]
        
        OUTPUT:
        Returns a tuple containing:
         - endDHparameters: The main DH parameter array.
         - jointspcDHparameters: The joint-specific DH parameter array.
        """
        # Construct the main DH parameters array exactly as in the forward function.
        endDHparameters = np.array([
            [0,      -pi/2,   0.333,     q[0]],          # Joint 1
            [0,       pi/2,     0,        q[1]],          # Joint 2
            [0.0825,  pi/2,   0.3160,    q[2]],          # Joint 3
            [-0.08250, -pi/2,  0,        q[3]],          # Joint 4
            [0,       pi/2,   0.384,     q[4]],          # Joint 5
            [0.088,   pi/2,    0.0,      q[5]],          # Joint 6
            [0,       0.0,    0.21,      q[6] - (pi/4)]  # Joint 7 (with offset)
        ])

        # Construct the joint-specific DH parameters array exactly as in the forward function.
        jointspcDHparameters = np.array([
            [0, 0, 0.141, q[0]], 
            [0, 0, 0,       q[1]],
            [0, 0, 0.195,   q[2]],
            [0, 0, 0,       q[3]],
            [0, 0, 0.125,   q[4]],
            [0, 0, -0.015,  q[5]],
            [0, 0, 0.051,   q[6]]
        ])

        # Return both arrays so they can be used elsewhere.
        return endDHparameters, jointspcDHparameters

    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        # STUDENT CODE HERE: This is a function needed by lab 2
        z_axes = [] # to store Z-axis (rotation axis) for each joint based in world frame
        endDHparameters, jointspcDHparameters = self.get_dh_parameters(q)
        z_axes = np.zeros((3, 7))
        T_current = np.identity(4)
        for i in range(7):
            # The z-axis of the current frame is the third column of T_current.
            z_axes[:, i] = T_current[:3, 2]
            mod_transform = self.dh_transform(*jointspcDHparameters[i])
            T_joint = T_current @ mod_transform

            main_transform = self.dh_transform(*endDHparameters[i])
            T_current = T_current @ main_transform

        return z_axes
    
    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
   
    # q = np.array([ pi/2, 0,  pi/4, -pi/2, -pi/2, pi/2,    0 ]) # config 1 
    # q = np.array([ 0,    0, -pi/2, -pi/4,  pi/2, pi,   pi/4 ]) # config 2
    # q = np.array([pi/2, pi/2, 0, pi/2, 0, pi/2, 0])  # Folded Configuration  (config 3) look at joint 1 and see if there might be something wrong in DH?

    
     # q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
    q = np.array([0, 0, 0, 0, 0, 0, 0]) # try out 0 position
    joint_positions, T0e = fk.forward(q)
    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
