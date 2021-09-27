import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseButton
from math import sin, cos

def forKin(theta, L):
    # NOTE: A complex number based forward kinematics
    # From Martin's 340 demo code.
    f = 0
    t = 0
    pos = np.zeros((3, 2))
    for idx in range(len(L)):
        # ####################
        # Forward Kinematics Here
        t = t + theta[idx]
        p = f + (np.complex(np.cos(t), np.sin(t))) * L[idx]

        # Grab the descriptors/pos of each joints
        pos[idx] = np.array([np.real(f), np.imag(f)])
        pos[idx+1] = np.array([np.real(p), np.imag(p)])
        f = p
    return pos

def jacobian_a_(theta, 
                L):
    """Get analytical jacobian.
    """
    J = np.zeros((2, 2), dtype=np.float32)
    J[0, 0] = -L[0] * sin(theta[0]) - L[1] * sin(theta[0] + theta[1])
    J[0, 1] = -L[1] * sin(theta[0] + theta[1])
    J[1, 0] = L[0] * cos(theta[0]) + L[1] * cos(theta[0] + theta[1])
    J[1, 1] = L[1] * cos(theta[0] + theta[1])
    return J

def jacobian_cd_(theta,
                 L,
                 alpha: float = 1e-8):
    """Get Jacobian from central difference.
    """
    J = np.asmatrix(np.zeros((2, 2), dtype=np.float32))
    J[0,:] = 0

    theta = np.asmatrix(theta)
    for i in range(J.shape[0]):
        alpha_ = np.asmatrix(np.zeros((2, 1), dtype=np.float32))
        alpha_[i, :] = alpha

        # Central difference, w.r.t theta_i
        j = (forKin(theta + alpha_, L)[-1] - forKin(theta - alpha_, L)[-1]) / (2 * alpha)
        j = np.asmatrix(j).T
        J[:,i] = j  # column

    return J

def invKin_v1(theta,
              L,
              pos1,
              method: dict,
              n_iters: int = 100, 
              threshold: float = 1e-2):
    """Inverse Kinematics
    Given desired pos, get joint angles.
    Version 1: Newton's method.
    """
    theta = np.asmatrix(theta)
    for _ in range(n_iters):
        # Analytical
        if method['jacobian'] == 'a':
            J = jacobian_a_(theta, L)
        # Or central difference
        else:
            J = jacobian_cd_(theta, L)

        pos0 = forKin(theta, L)[-1, :]
        delta_p = np.asmatrix(pos0 - pos1).T
        s = np.linalg.solve(-J, delta_p)
        theta += s
        if s.all() < threshold:
            break
    return theta

def invKin_v2(theta,
              L,
              pos1,
              method: dict,
              n_iters: int = 10, 
              threshold: float = 1e-2):
    """Inverse Kinematics
    Given desired pos, get joint angles.
    Version 2: Broyden's method.
    """
    theta = np.asmatrix(theta)
    # Analytical
    if method['jacobian'] == 'a':
        B = jacobian_a_(theta, L)
    # Or central difference
    else:
        B = jacobian_cd_(theta, L)

    for _ in range(n_iters):
        pos0 = forKin(theta, L)[-1, :]
        delta_p = np.asmatrix(pos0 - pos1).T
        s = np.linalg.pinv(-B) * delta_p
        theta += s
        if s.all() < threshold:
            break

        # Update approx jacobian
        y = np.asmatrix(forKin(theta, L)[-1, :] - pos0).T   # delta_p current iter
        B += ((y - B * s) * s.T) / (s.T * s)

    return theta

class Arm2D:
    def __init__(self, 
                 theta: np.ndarray = None,
                 L: np.ndarray = np.array([0.5, 0.5]).T,
                 method: dict = {'invKin': 'broyden', 
                                 'jacobian': 'cd'}
                ):
        self.L = L  # Link length
        self.theta = theta  # Joint angle
        self.method = method
        self._constraint_fig()

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

        # Move robot to desired coordinate using inverse kinematics
        if self.method['invKin'] == 'broyden':
            self.theta = invKin_v2(self.theta, self.L, self.p, method=self.method)
        else:
            self.theta = invKin_v1(self.theta, self.L, self.p, method=self.method)

        # Draw
        self.plot(event)

    def plot(self, 
             event=None):
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
        pos = np.zeros((3, 2))
        for idx in range(len(self.L)):
            # ####################
            # Forward Kinematics Here
            t = t + self.theta[idx]
            p = f + (np.complex(np.cos(t), np.sin(t))) * self.L[idx]

            # Grab the descriptors/pos of each joints
            pos[idx] = np.array([np.real(f), np.imag(f)])
            pos[idx+1] = np.array([np.real(p), np.imag(p)])
            # ####################

            plotSeg(f, p)
            f = p

        # Darken joints
        self.ax.scatter(pos[:,0], pos[:,1], c='orange')

        # Draw clicked point
        if event is not None:
            print('Clicked Point:', self.p)
            self.ax.scatter(self.p[0], self.p[1], c='blue')
            event.canvas.draw()
        else:
            plt.show()
        return