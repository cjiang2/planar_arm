from math import sin, cos, pi

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseButton

from .camera import plotCamera
from . import basic_servoing, visual_servoing

# --------------------
# Forward Kinematics for 3D Arm
# --------------------

def DH_matrix(alpha_i_1: float, 
              a_i_1: float,
              d_i: float,
              theta_i):
    """Get Denavit–Hartenberg transformation matrix.
    """
    return np.matrix([[cos(theta_i), -sin(theta_i)*cos(alpha_i_1), sin(theta_i)*cos(alpha_i_1), a_i_1*cos(theta_i)], 
                      [sin(theta_i), cos(theta_i) * cos(alpha_i_1), -cos(theta_i)*sin(alpha_i_1), -a_i_1 * sin(theta_i)],
                      [0, sin(alpha_i_1), cos(alpha_i_1), d_i],
                      [0, 0, 0, 1],
                     ])

def forKin(theta, 
           L, 
           p0=np.array([0.0, 0.0, 0.0, 1])):
    """Forward kinematics for a 3-joint robot.
    Using DH parameters.
    """
    # Twist alpha_i, link length a_i, link offset d_i, joint angle theta_i
    dh_table = [[pi/2, 0, L[0], np.asscalar(theta[0])], 
                [0, L[1], 0, np.asscalar(theta[1])],
                [0, L[2], 0, np.asscalar(theta[2])]]

    # Transformation
    T0_1 = DH_matrix(dh_table[0][0], dh_table[0][1], dh_table[0][2], dh_table[0][3])
    T1_2 = DH_matrix(dh_table[1][0], dh_table[1][1], dh_table[1][2], dh_table[1][3])
    T2_3 = DH_matrix(dh_table[2][0], dh_table[2][1], dh_table[2][2], dh_table[2][3])
    
    P0 = np.matrix([p0[0], p0[1], p0[2], p0[3]]).T
    p1 = np.asarray(T0_1 * P0).squeeze(1)
    p2 = np.asarray(T0_1 * T1_2 * P0).squeeze(1)
    p3 = np.asarray(T0_1 * T1_2 * T2_3 * P0).squeeze(1)

    pos = np.stack([p0, p1, p2, p3], axis=1)
    return pos   # w/ homogeneous term (4, dof)


# --------------------
# Simulated 2D Pose Tracking
# --------------------

class PoseTrack(object):
    """Fake visual tracker to track pose of the end effector.
    """
    def __init__(self, 
                 cams: list,
                 dof: int):
        self.cams = cams
        self.dof = dof

    def track(self, 
              pos: np.ndarray):
        """Return 2D pos in all camera views.
        NOTE: Fake tracker will return the ideal projected 2D
        coordinates using known cameras.
        """
        assert pos.shape[0] == 4
        if len(pos.shape) == 1:
            pos = np.expand_dims(pos, axis=1)

        x = np.zeros((len(self.cams), 2))
        for i, cam in enumerate(self.cams):
            x_cam = cam(pos).squeeze(1)
            x[i,:] = x_cam

        return x.reshape(-1)


class Arm3D:
    def __init__(self, 
                 theta: np.ndarray = None,
                 L: np.ndarray = np.array([0.5, 0.5, 0.5]),
                 method: dict = {'invKin': 'broyden', 
                                 'jacobian': 'cd',
                                 'visual_servoing': True},
                 cams: list = None,
                ):
        self.theta = theta
        self.L = L
        self.p0 = np.array([0.0, 0.0, 0.0, 1])

        self._constraint_fig()
        self.cams = cams
        if cams is not None and len(cams) > 0:
            self._constraint_fig_cam()

        self.set_method(method)

    def set_method(self, 
                   method: dict):
        self.method = method
        self.forKin_func = forKin

        # Use visual servoing
        if method['visual_servoing']:
            self.jacobian_func = visual_servoing.jacobian_cd_
            self.invkin_func = visual_servoing.invKin_vs
            self.pose_tracker = PoseTrack(cams=self.cams, dof=len(self.theta))

        # Classic pose-based controls
        else:
            if method['invKin'] == 'newton':
                self.invkin_func = basic_servoing.invKin_v1
            else:
                self.invkin_func = basic_servoing.invKin_v2

            if method['jacobian'] == 'a':
                self.jacobian_func = basic_servoing.jacobian_a_
            else:
                self.jacobian_func = basic_servoing.jacobian_cd_

        print(str(self.forKin_func))
        print(str(self.invkin_func))
        print(str(self.jacobian_func))
    
    def _constraint_fig(self):
        """To make figure look nicer.
        """
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')
        self.cid = self.fig.canvas.mpl_connect('key_press_event', self)

    def _constraint_fig_cam(self):
        """To make figure look nicer.
        For camera projection.
        """
        self.fig_cam = plt.figure(2)
        self.axes_cam = []
        for i in range(len(self.cams)):
            self.axes_cam.append(self.fig_cam.add_subplot(1, len(self.cams), i + 1))
            #if 'PinholeCam' not in str(self.cams[i]):
            self.axes_cam[i].axis('equal')

    def __call__(self, 
                 event):
        # Current end effector pos
        p_end = self.forKin_func(self.theta, self.L, self.p0)[:,-1]
        
        # Left mouse click saves coordinates
        if event.key == 'left':
            p_end[0] = p_end[0] - 0.1
        elif event.key == 'right':
            p_end[0] = p_end[0] + 0.1
        elif event.key == 'up':
            p_end[2] = p_end[2] + 0.1
        elif event.key == 'down':
            p_end[2] = p_end[2] - 0.1
        elif event.key == 'z':
            p_end[1] = p_end[1] - 0.1
        elif event.key == 'c':
            p_end[1] = p_end[1] + 0.1

        # Move robot to desired coordinate using inverse kinematics
        # Visual Servoing
        if self.method['visual_servoing']:
            # For Visual Servoing, end effector goal is specificed
            # by 2D pos in camera view
            pos_end = self.pose_tracker.track(p_end)
            self.theta = self.invkin_func(self.theta, self.L, pos_end,
                                          forKin_func=self.forKin_func,
                                          pose_tracker=self.pose_tracker)

        # Basic Servoing using 3D pos
        else:
            self.theta = self.invkin_func(self.theta, self.L, p_end[:-1], 
                                          forKin_func=self.forKin_func,
                                          jacobian_func=self.jacobian_func)

        # Draw
        self.plot(event)


    def plot(self, 
             event=None):
        """Plot the 3d robot joints.
        """
        # Clear figure 1st
        self.ax.clear()
        lim_range = 2.0
        self.ax.set_xlim([-lim_range, lim_range])
        self.ax.set_ylim([-lim_range, lim_range])
        self.ax.set_zlim([0, lim_range])
        self.ax.set_xlabel('$X$')
        self.ax.set_ylabel('$Y$')
        self.ax.set_zlabel('$Z$')

        # Current segments
        pos = self.forKin_func(self.theta, self.L, self.p0)

        # Plot the new pos
        self.ax.scatter(pos[0,:], pos[1,:], pos[2,:])
        self.ax.plot(pos[0,:], pos[1,:], pos[2,:], 'k')
        

        # If camera project is needed
        if self.cams is not None:
            for i, cam in enumerate(self.cams):
                ax_cam = self.axes_cam[i]
                ax_cam.clear()

                # Camera projection
                p = cam(pos)

                # Plot
                ax_cam.invert_yaxis()
                ax_cam.xaxis.tick_top()
                ax_cam.set_title(str(cam))
                ax_cam.scatter(p[0,:], p[1,:])

                plotCamera(cam, self.ax)
                #self.ax.scatter(cam.C[0], cam.C[1], cam.C[2], c='orange')
        
        if event is not None:
            event.canvas.draw()
            if self.cams is not None:
                self.fig_cam.canvas.draw()

        else:
            plt.show()
