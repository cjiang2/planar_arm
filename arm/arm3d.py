import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos

def forKin_v1(theta, 
              L, 
              p0=np.array([0.0, 0.0, 0.0])):
    """From Martin's robotics lecture. 
    Forward kinematics for a 3-joint robot.
    """
    # Joint 01
    Rxy1 = np.matrix([[cos(theta[0]), sin(theta[0]), 0],
                      [-sin(theta[0]), cos(theta[0]), 0], 
                      [0, 0, 1],
                     ])
    #print(Rxy1.shape)

    p1 = Rxy1 * np.matrix([[L[0], 0, 0]]).T
    #print(p1.shape)
    p1 = np.asarray(p1).squeeze(1) + p0
    #print(p1, p1.shape)

    # Joint 12
    Rxz2 = np.matrix([[cos(theta[1]), 0, sin(theta[1])],
                      [0, 1, 0],
                      [-sin(theta[1]), 0, cos(theta[1])], 
                     ])
    p2 = Rxy1 * Rxz2 * np.matrix([[L[1], 0, 0]]).T
    p2 = np.asarray(p2).squeeze(1) + p1
    #print(p2, p2.shape)

    # Joint 23
    Rxz3 = np.matrix([[cos(theta[2]), 0, sin(theta[2])],
                      [0, 1, 0],
                      [-sin(theta[2]), 0, cos(theta[2])], 
                     ])
    p3 = Rxy1 * Rxz2 * Rxz3 * np.matrix([[L[2], 0, 0]]).T
    p3 = np.asarray(p3).squeeze(1) + p2
    #print(p3, p3.shape)

    pos = np.stack([p0, p1, p2, p3], axis=0)
    #print(pos.shape)
    return pos

def forKin_v2(theta, 
              L, 
              p0 = np.array([0.0, 0.0, 0.0, 1])):
    """From Martin's robotics lecture. 
    Forward kinematics for a 3-joint robot.
    Version 2: Convert into homogeneous transformation.
    """
    pos = None
    return pos


class Arm3D:
    def __init__(self, 
                 L: np.ndarray = np.array([0.5, 0.5, 0.5]),
                ):
        self.L = L
        self._constraint_fig()
    
    def _constraint_fig(self, 
                        lim_range: float = 2.0):
        """To make figure look nicer.
        """
        self.fig = plt.figure()
        self.ax = self.fig.gca(projection='3d')
        self.ax.set_xlim([0.0, lim_range])
        self.ax.set_ylim([-lim_range, lim_range])
        self.ax.set_zlim([-lim_range, 0.0])

    def _forward_kine(self, 
                      theta):
        """Forward kinematics.
        Given fixed length joints and joint angle, compute pos.
        """
        # Enforece joint 0 at origin
        pos = forKin_v1(theta, self.L)
        print(pos)
        #pos = forKin_v2(theta, self.L)
        return pos

    def _inverse_kine(self):
        """Inverse kinematics.
        """
        return
    
    def plot(self, 
             theta):
        """Plot the 3d robot joints.
        """
        # Home
        pos = self._forward_kine(theta)

        # Plot the new pos
        self.ax.scatter(pos[:,0], pos[:,1], pos[:,2])
        self.ax.plot(pos[:,0], pos[:,1], pos[:,2], 'k')
        
        plt.show()