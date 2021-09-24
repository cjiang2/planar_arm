import numpy as np
import matplotlib.pyplot as plt

from arm.arm2d import Arm2D

if __name__ == '__main__':
    arm_env = Arm2D()

    # Start with a random angle
    t = np.random.uniform(size=(2, 1))

    # Plot
    arm_env.plot(t)