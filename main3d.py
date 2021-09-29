import numpy as np
import matplotlib.pyplot as plt

from arm.arm3d import Arm3D

if __name__ == '__main__':
    # Start with a random angle
    t = np.random.uniform(size=(3, 1))
    arm_env = Arm3D(t)
    arm_env.plot()

    
    