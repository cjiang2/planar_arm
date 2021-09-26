import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseButton

class Arm2D:
    def __init__(self, 
                 theta: np.ndarray = None,
                 L: np.ndarray = np.array([0.5, 0.5]).T):
        self.L = L  # Link length
        self.theta = theta  # Joint angle
        self._constraint_fig()

        self.pos = np.zeros((3, 2))   # Pos of joints

    def _constraint_fig(self):
        """To make figure look nicer.
        """
        self.p = None   # Click point from event / desired end effector position
        self.fig = plt.figure()
        self.ax = self.fig.gca()
        self.ax.axis('equal')
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self)

    def __call__(self, 
                 event):
        # Left mouse click saves coordinates
        if event.button is MouseButton.LEFT:
            self.p = np.array([event.xdata, event.ydata])

        # ####################
        # Write Plotting, forward and inverse kinematics here
        self.plot()

        # ####################

    def _inverse_kine(self):
        """Inverse kinematics.
        """
        return
    
    def plot(self):
        """Plot the 2D planar manipulator.
        """
        def mycircle(rad,
                     px,
                     py):
            t = np.linspace(0, 2 * np.pi, 10)
            self.ax.plot(rad * np.cos(t) + px, rad * np.sin(t) + py, 'k')
            return

        def plotSeg(from_, 
                    to_):
            d = to_ - from_
            l = np.linalg.norm(d)
            r1 = l/10
            r2 = l/15
            r3 = l/7
            d = d/l

            mycircle(l/10, np.real(from_), np.imag(from_))
            mycircle(l/15, np.real(to_), np.imag(to_))

            pts = np.array([from_ + np.complex(0, r1)*d, 
                            to_ + np.complex(0, r2)*d])
            self.ax.plot(np.real(pts), np.imag(pts), 'k')

            pts = np.array([from_ - np.complex(0, r1)*d, 
                            to_ - np.complex(0, r2)*d])
            self.ax.plot(np.real(pts), np.imag(pts), 'k')

            # The axis
            pts = np.array([to_, to_ + r3*d])
            self.ax.plot(np.real(pts), np.imag(pts), 'r')
            pts = np.array([to_, to_ + np.complex(0, r3) * d])
            self.ax.plot(np.real(pts), np.imag(pts), 'g')

        # Clear figure 1st
        self.ax.clear()

        # Plot an invisible workspace
        ltot = np.sum(self.L)
        self.ax.plot([-ltot, ltot, ltot, -ltot], [-ltot, -ltot, ltot, ltot], 'w')

        # Draw each segment
        # NOTE: A complex number based forward kinematics is here
        f = 0
        t = 0
        for idx in range(len(self.L)):
            # ####################
            # Forward Kinematics Here
            t = t + self.theta[idx]
            p = f + (np.complex(np.cos(t), np.sin(t))) * self.L[idx]

            # Grab the descriptors/pos of each joints
            self.pos[idx] = np.array([np.real(f), np.imag(f)])
            self.pos[idx+1] = np.array([np.real(p), np.imag(p)])
            # ####################

            plotSeg(f, p)
            f = p

        # Draw clicked point
        if self.p is not None:
            print('Clicked Point:', self.p)
            self.ax.scatter(self.p[0], self.p[1], c='blue')

        # Darken joints
        self.ax.scatter(self.pos[:,0], self.pos[:,1], c='orange')

        plt.show()
        return