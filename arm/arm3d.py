import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos
from matplotlib.backend_bases import MouseButton

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

    p1 = Rxy1 * np.matrix([[L[0], 0, 0]]).T
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
    return pos   # Remove homogeneous term

def forKin(theta, 
           L, 
           p0 = np.array([0.0, 0.0, 0.0, 1])):
    """From Martin's robotics lecture. 
    Forward kinematics for a 3-joint robot.
    Version 2: Convert into homogeneous transformation.
    """
    # Joint 01
    P0 = np.asmatrix(p0).T
    Rxy1 = np.matrix([[cos(theta[0]), sin(theta[0]), 0, L[0]],
                      [-sin(theta[0]), cos(theta[0]), 0, 0], 
                      [0, 0, 1, 0],
                      [0, 0, 0, 1],
                     ])
    p1 = Rxy1 * P0
    p1 = np.asarray(p1).squeeze(1)
    #print(p1, p1.shape)

    # Joint 12
    Rxz2 = np.matrix([[cos(theta[1]), 0, sin(theta[1]), L[1]],
                      [0, 1, 0, 0],
                      [-sin(theta[1]), 0, cos(theta[1]), 0],
                      [0, 0, 0, 1] 
                     ])
    p2 = Rxy1 * Rxz2 * P0
    p2 = np.asarray(p2).squeeze(1)
    #print(p2, p2.shape)

    # Joint 23
    Rxz3 = np.matrix([[cos(theta[2]), 0, sin(theta[2]), L[2]],
                      [0, 1, 0, 0],
                      [-sin(theta[2]), 0, cos(theta[2]), 0],
                      [0, 0, 0, 1], 
                     ])
    p3 = Rxy1 * Rxz2 * Rxz3 * P0
    p3 = np.asarray(p3).squeeze(1)
    #print(p3, p3.shape)

    pos = np.stack([p0, p1, p2, p3], axis=0)
    #print(pos.shape)
    return pos[:,:-1]   # Remove homogeneous term

def jacobian_cd_(theta,
                 L,
                 alpha: float = 1e-8,
                 forKin_func = forKin):
    """Get Jacobian from central difference.
    """
    J = np.asmatrix(np.zeros((3, len(theta)), dtype=np.float32))
    J[0,:] = 0

    theta = np.asmatrix(theta)
    for i in range(J.shape[0]):
        alpha_ = np.asmatrix(np.zeros((len(theta), 1), dtype=np.float32))
        alpha_[i, :] = alpha

        # Central difference, w.r.t theta_i
        # Remove homogeneous term
        j = (forKin_func(theta + alpha_, L)[-1,:] - forKin_func(theta - alpha_, L)[-1,:]) / (2 * alpha)
        j = np.asmatrix(j).T
        J[:,i] = j  # column

    return J

def jacobian_a_(theta,
                L, 
                p0 = np.array([0.0, 0.0, 0.0, 1])):
    """Get the analytical Jacobian.
    """
    # Prep jacobian
    J = np.asmatrix(np.zeros((3, len(theta)), dtype=np.float32))

    # Geometric Forward Kinematics
    P0 = np.asmatrix(p0).T
    T0 = np.matrix([[1, 0, 0, L[0]],
                    [0, 1, 0, 0], 
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                   ])
    Rxy1 = np.matrix([[cos(theta[0]), sin(theta[0]), 0, 0],
                      [-sin(theta[0]), cos(theta[0]), 0, 0], 
                      [0, 0, 1, 0],
                      [0, 0, 0, 1],
                     ])

    T1 = np.matrix([[1, 0, 0, L[1]],
                    [0, 1, 0, 0], 
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                   ])
    Rxz2 = np.matrix([[cos(theta[1]), 0, sin(theta[1]), 0],
                      [0, 1, 0, 0],
                      [-sin(theta[1]), 0, cos(theta[1]), 0],
                      [0, 0, 0, 1] 
                     ])

    T2 = np.matrix([[1, 0, 0, L[2]],
                    [0, 1, 0, 0], 
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                   ])
    Rxz3 = np.matrix([[cos(theta[2]), 0, sin(theta[2]), 0],
                      [0, 1, 0, 0],
                      [-sin(theta[2]), 0, cos(theta[2]), 0],
                      [0, 0, 0, 1], 
                     ])

    # Derivatives    
    dRxy1 = np.matrix([[-sin(theta[0]), cos(theta[0]), 0, 0],
                       [-cos(theta[0]), -sin(theta[0]), 0, 0], 
                       [0, 0, 0, 0],
                       [0, 0, 0, 0],
                      ])
    dRxz2 = np.matrix([[-sin(theta[1]), 0, cos(theta[1]), 0],
                       [0, 0, 0, 0],
                       [-cos(theta[1]), 0, -sin(theta[1]), 0],
                       [0, 0, 0, 0] 
                      ])
    dRxz3 = np.matrix([[-sin(theta[2]), 0, cos(theta[2]), 0],
                       [0, 0, 0, 0],
                       [-cos(theta[2]), 0, -sin(theta[2]), 0],
                       [0, 0, 0, 0], 
                      ])

    # Calculate by column
    df_dtheta1 = T0 * dRxy1 * T1 * Rxz2 * T2 * Rxz3 * P0
    df_dtheta2 = T0 * Rxy1 * T1 * dRxz2 * T2 * Rxz3 * P0
    df_dtheta3 = T0 * Rxy1 * T1 * Rxz2 * T2 * dRxz3 * P0

    J[:,0] = df_dtheta1[:-1,:]
    J[:,1] = df_dtheta2[:-1,:]
    J[:,2] = df_dtheta3[:-1,:]

    return J

def invKin_v1(theta,
              L,
              pos1,
              n_iters: int = 1000, 
              threshold: float = 1e-2,
              forKin_func = forKin,
              jacobian_func = jacobian_cd_):
    """Inverse Kinematics
    Given desired pos, get joint angles.
    Version 1: Newton's method.
    """
    theta = np.asmatrix(theta)
    for _ in range(n_iters):
        if "jacobian_cd_" in str(jacobian_func):
            B = jacobian_func(theta, L, forKin_func=forKin_func)
        else:
            B = jacobian_func(theta, L)

        pos0 = forKin_func(theta, L)[-1, ]
        delta_p = np.asmatrix(pos0 - pos1).T
        
        s = np.linalg.solve(-J, delta_p)
        theta += s
        if s.all() < threshold:
            break
    return theta

def invKin_v2(theta,
              L,
              pos1,
              n_iters: int = 10, 
              threshold: float = 1e-2,
              forKin_func = forKin,
              jacobian_func = jacobian_cd_):
    """Inverse Kinematics
    Given desired pos, get joint angles.
    Version 2: Broyden's method.
    """
    theta = np.asmatrix(theta)
    # Initial estimation of the jacobian
    if "jacobian_cd_" in str(jacobian_func):
        B = jacobian_func(theta, L, forKin_func=forKin_func)
    else:
        B = jacobian_func(theta, L)

    for _ in range(n_iters):
        pos0 = forKin_func(theta, L)[-1, :]
        delta_p = np.asmatrix(pos0 - pos1).T
        s = np.linalg.pinv(-B) * delta_p
        theta += s
        if s.all() < threshold:
            break

        # Update approx jacobian
        y = np.asmatrix(forKin_func(theta, L)[-1, :] - pos0).T   # delta_p current iter
        B += ((y - B * s) * s.T) / (s.T * s)

    return theta


class Arm3D:
    def __init__(self, 
                 theta: np.ndarray = None,
                 L: np.ndarray = np.array([0.5, 0.5, 0.5]),
                 method: dict = {'invKin': 'broyden', 
                                 'jacobian': 'cd',
                                 'forKin': 'forKin'},
                ):
        self.theta = theta
        self.L = L
        self.p0 = np.array([0.0, 0.0, 0.0, 1])
        self._constraint_fig()
    
    def _constraint_fig(self):
        """To make figure look nicer.
        """
        self.fig = plt.figure()
        self.ax = self.fig.gca(projection='3d')
        self.cid = self.fig.canvas.mpl_connect('key_press_event', self)

    def __call__(self, 
                 event):
        # Current pos
        self.p = forKin(self.theta, self.L, self.p0)[-1,:]
        
        # Left mouse click saves coordinates
        if event.key == 'left':
            self.p[0] = self.p[0] - 0.1
        elif event.key == 'right':
            self.p[0] = self.p[0] + 0.1
        elif event.key == 'up':
            self.p[2] = self.p[2] + 0.1
        elif event.key == 'down':
            self.p[2] = self.p[2] - 0.1
        elif event.key == 'a':
            self.p[1] = self.p[1] - 0.1
        elif event.key == 'd':
            self.p[1] = self.p[1] + 0.1

        # Move robot to desired coordinate using inverse kinematics
        self.theta = invKin_v2(self.theta, self.L, self.p)

        # Draw
        self.plot(event)

    def plot(self, 
             event=None):
        """Plot the 3d robot joints.
        """
        # Clear figure 1st
        self.ax.clear()
        lim_range = 2.0
        self.ax.set_xlim([0.0, lim_range])
        self.ax.set_ylim([-lim_range, lim_range])
        self.ax.set_zlim([-lim_range, 0.0])
        self.ax.set_xlabel('$X$')
        self.ax.set_ylabel('$Y$')
        self.ax.set_zlabel('$Z$')

        # Current segments
        pos = forKin(self.theta, self.L, self.p0)

        # Plot the new pos
        self.ax.scatter(pos[:,0], pos[:,1], pos[:,2])
        self.ax.plot(pos[:,0], pos[:,1], pos[:,2], 'k')
        
        if event is not None:
            event.canvas.draw()
        else:
            plt.show()