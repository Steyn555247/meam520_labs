from lib.calculateFK import FK
from core.interfaces import ArmController
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fk = FK()

# the dictionary below contains the data returned by calling arm.joint_limits()
limits = [
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -1.7628, 'upper': 1.7628},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -3.0718, 'upper': -0.0698},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -0.0175, 'upper': 3.7525},
    {'lower': -2.8973, 'upper': 2.8973}
 ]

# TODO: create plot(s) which visualize the reachable workspace of the Panda arm,
# accounting for the joint limits.
#
# We've included some very basic plotting commands below, but you can find
# more functionality at https://matplotlib.org/stable/index.html
num_samples = 10000 #number of samples
q_samples = np.array([
    np.random.uniform(joint['lower'], joint['upper'], num_samples) for joint in limits
]).T

end_effector_positions = np.array([fk.forward(q)[1][:3, 3] for q in q_samples])


fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

# TODO: update this with real results
ax.scatter(end_effector_positions[:, 0], end_effector_positions[:, 1], end_effector_positions[:, 2], c='blue', s=1, alpha = 0.5)

ax.set_xlabel("X-axis M")
ax.set_ylabel("Y-axis M")
ax.set_zlabel("Z-axis M")


plt.show()
