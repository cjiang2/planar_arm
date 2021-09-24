import numpy as np
import matplotlib.pyplot as plt

class Arm3D:
    def __init__(self, 
                 L: np.ndarray = np.array([0.5, 0.5, 0.5]),
                ):
        self.L = L
        self._constraint_fig()
    
    def _constraint_fig(self, 
                        lim_range: float = 4.0):
        """To make figure look nicer.
        """
        self.fig = plt.figure()
        self.ax = self.fig.gca(projection='3d')
        self.ax.set_xlim([-lim_range, lim_range])
        self.ax.set_ylim([0.0, lim_range])
        self.ax.set_zlim([0.0, lim_range])

    def _forward_kine(self):
        """Forward kinematics.
        """
        return

    def _inverse_kine(self):
        """Inverse kinematics.
        """
        return
    
    def plot(self):
        """Plot the 3d robot joints.
        """
        # Home
        # ####################
        # TODO: HARDCODED
        # ####################
        p = np.array([3.0, 3.0, 3.0])  # End effector position
        p0 = np.array([0.0, 0.0, 0.0])   # Joint 0 will always be 0

        p1 = np.array([1.0, 1.0, 1.0])
        p2 = np.array([2.0, 2.0, 2.0])

        pos = np.stack([p0, p1, p2, p], axis=0)
        # ####################

        self.ax.scatter(pos[:,0], pos[:,1], pos[:,2])
        self.ax.plot(pos[:,0], pos[:,1], pos[:,2], 'k')
        
        plt.show()