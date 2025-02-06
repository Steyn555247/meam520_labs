import numpy as np
from math import pi, sin, cos

class FK():
    # def __init__(self):
    #     # DH Parameters: (theta, d, a, alpha) following Craig's standard DH convention.
    #     # These values come from the lab handout.
    #     #format rows as [a, alpha, d, theta]
        

    def dh_transform( self, a, alpha, d, theta):
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
            [0,      pi/2,     0.0,    q[1]],       # Joint 2
            [0.0825, pi/2,   0.3160,    q[2]],        # Joint 3
            [-0.0825, -pi/2, 0,  q[3]],        # Joint 4
            [0,      pi/2,  0.384, q[4]],        # Joint 5
            [0.088,   pi/2,     0.0,     q[5]],        # Joint 6
            [0,      0.0,     0.21,   q[6]-(pi/4)]       # Joint 7
        ])

        jointspcDHparameters=np.array([
            [0, 0, 0.141, q[0]],
            [0, 0, 0, q[1]],
            [0, 0, 0.195, q[2]],
            [0, 0, 0, q[3]],
            [0, 0, 0.125, q[4]],
            [0, 0, 0.015, q[5]],
            [0, 0, 0.051, q[6]]
        ])
       
        jointPositions = np.zeros((8, 3))
        T0e = np.identity(4)

        for i in range(7):
            modtransform= self.dh_transform(*jointspcDHparameters[i])
            maintransform = self.dh_transform(*endDHparameters[i])
            Tmod = T0e @ modtransform
            jointPositions[i] = Tmod[:3, 3].tolist()
            T0e = T0e @ maintransform

            jointPositions[7] = T0e[:3, 3].tolist()

        


        # Your code ends here

        return jointPositions, T0e

    # feel free to define additional helper methods to modularize your solution for lab 1

    
    # This code is for Lab 2, you can ignore it ofr Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
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
    q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
# q = np.array([0, 0, 0, 0, 0, 0, 0])
    joint_positions, T0e = fk.forward(q)
    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)

