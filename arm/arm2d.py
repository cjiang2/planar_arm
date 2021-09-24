import numpy as np
import matplotlib.pyplot as plt

class Arm2D:
    def __init__(self, 
                 L: np.ndarray = np.array([0.5, 0.5]).T):
        self._constraint_fig()
        self.L = L  # Link length

    def _constraint_fig(self):
        """To make figure look nicer.
        """
        self.fig = plt.figure()
        self.ax = self.fig.gca()
        self.ax.axis('equal')

    def _forward_kine(self, 
                      angles):
        """Forward Kinematics.
        Given link lengths, and joint angles, compute cartesian 
        coordinates for each joint.
        """
        # Draw each segment
        f = 0
        t = 0
        fs, ps = [], []
        for idx in range(len(self.L)):
            t = t + angles[idx]
            p = f + (np.complex(np.cos(t), np.sin(t))) * self.L[idx]
            fs.append(f)
            ps.append(p)
            f = p
        return fs, ps

    def _inverse_kine(self):
        """Inverse kinematics.
        """
        return
    
    def plot(self, 
             angles):
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

        # Plot an invisible workspace
        ltot = np.sum(self.L)
        self.ax.plot([-ltot, ltot, ltot, -ltot], [-ltot, -ltot, ltot, ltot], 'w')

        fs, ps = self._forward_kine(angles)
        for i in range(len(fs)):
            plotSeg(fs[i], ps[i])

        plt.show()
        return