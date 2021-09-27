import numpy as np
import matplotlib.pyplot as plt

from arm.arm2d import Arm2D

if __name__ == '__main__':
    # Start with a random angle
    t = np.random.uniform(size=(2, 1))
    arm_env = Arm2D(t, method='newton')

    arm_env.plot()