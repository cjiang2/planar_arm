import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def get_rotation_matrix(theta_x: float = 0.0, 
                        theta_y: float = 0.0,
                        theta_z: float = 0.0):
    # Roll
    Rx = np.matrix([[1, 0, 0],
                    [0, np.cos(theta_x), -np.sin(theta_x)],
                    [0, np.sin(theta_x), np.cos(theta_x)]])

    # Pitch
    Ry = np.matrix([[np.cos(theta_y), 0, np.sin(theta_y)],
                    [0, 1, 0],
                    [-np.sin(theta_y), 0, np.cos(theta_y)]])

    # Yaw
    Rz = np.matrix([[np.cos(theta_z), -np.sin(theta_z), 0],
                    [np.sin(theta_z), np.cos(theta_z), 0],
                    [0, 0, 1]])


    return Rz * Ry * Rx


class Camera(object):
    def __init__(self, 
                 f: float):
        self.f = f

    def __call__(self, 
                 X: np.ndarray):
        raise NotImplemented


class PinholeCam(Camera):
    """Simple pinhole camera
    """
    def __init__(self, 
                 f: float,
                 px: float,
                 py: float,
                 C: np.ndarray,
                 theta: list = [0.0, 0.0, 0.0]):
        super().__init__(f)
        self.px, self.py = px, py
        self.C = C
        self.theta = theta

        self.K = np.matrix([[f, 0, px], 
                            [0, f, py],
                            [0, 0, 1]])
        self.R = get_rotation_matrix(theta[0], theta[1], theta[2])
        self.P = self.K * self.R * np.asmatrix(np.c_[np.eye(3), -np.asarray(C)])
    
    def __call__(self, 
                 X: np.ndarray):
        X = np.asmatrix(X)
        assert X.shape[0] == 4

        x = self.P * X
        x = x[:-1, :]
        return np.asarray(x)

    def __str__(self):
        return "PinholeCam\nf: {}, (px, py): ({}, {}), C: {}".format(self.f, 
                                                                     self.px, self.py, 
                                                                     self.C)

class CCDCam(Camera):
    def __init__(self, 
                 f: float,
                 mx: float,
                 my: float,
                 px: float,
                 py: float,
                 C: np.ndarray,
                 theta: list = [0.0, 0.0, 0.0]):
        super().__init__(f)
        self.mx, self.my = mx, my
        self.px, self.py = px, py
        self.C = C

        self.K = np.matrix([[f*mx, 0, mx*px], 
                            [0, f*my, my*py],
                            [0, 0, 1]])
        self.R = get_rotation_matrix(theta[0], theta[1], theta[2])
        self.P = self.K * self.R * np.asmatrix(np.c_[np.eye(3), -np.asarray(C)])

    def __call__(self, 
                 X: np.ndarray):
        X = np.asmatrix(X)
        assert X.shape[0] == 4

        x = self.P * X
        x = x[:-1, :]
        return np.asarray(x)

    def __str__(self):
        return "CCDCam\nf: {}, (mx, my): ({}, {}), (px, py): ({}, {}), C: {}".format(self.f, 
                                             self.mx, self.my,
                                             self.px, self.py, 
                                             self.C)

class OrthoProjCam(Camera):
    def __init__(self):
        super().__init__(f=None)
        self.P = np.matrix([[1, 0, 0, 0], 
                            [0, 1, 0, 0],
                            [0, 0, 0, 1]])

    def __call__(self, 
                 X: np.ndarray):
        X = np.asmatrix(X)
        assert X.shape[0] == 4

        x = self.P * X
        x = x[:-1, :]
        return np.asarray(x)


def plotCamera(cam,
               ax,
               scale=.5,
               depth=.5,
               ptColor='k',
               faceColor='grey'):
    """Plot the camera model with image plane.
    Reference from:
        https://github.com/muneebaadil/how-to-sfm/blob/master/tutorial/utils.py
    """
    # Generating camera coordinate axes
    R = np.asarray(cam.R)
    C = cam.C
    axes = np.zeros((3, 6))
    axes[0,1], axes[1,3], axes[2,5] = 1, 1, 1
      
    # Transforming to world coordinate system 
    R = np.asarray(R)
    axes = R.T.dot(axes) + C[:,np.newaxis]

    # Plotting axes
    ax.plot3D(xs=axes[0,:2], ys=axes[1,:2], zs=axes[2,:2],c='r')
    ax.plot3D(xs=axes[0,2:4], ys=axes[1,2:4], zs=axes[2,2:4],c='g')
    ax.plot3D(xs=axes[0,4:], ys=axes[1,4:], zs=axes[2,4:],c='b')

    # Generating 5 corners of camera polygon 
    pt1 = np.array([[0,0,0]]).T #camera centre
    pt2 = np.array([[scale, -scale, depth]]).T #upper right 
    pt3 = np.array([[scale, scale, depth]]).T #lower right 
    pt4 = np.array([[-scale, -scale, depth]]).T #upper left
    pt5 = np.array([[-scale, scale, depth]]).T #lower left
    pts = np.concatenate((pt1, pt2, pt3, pt4, pt5),axis=-1)
        
    # Transforming to world-coordinate system
    pts = R.T.dot(pts) + C[:,np.newaxis]
    ax.scatter3D(xs=pts[0,:], ys=pts[1,:], zs=pts[2,:], c=ptColor)
        
    # Generating a list of vertices to be connected in polygon
    verts = [[pts[:,0], pts[:,1], pts[:,2]], [pts[:,0], pts[:,2], pts[:,-1]],
             [pts[:,0], pts[:,-1], pts[:,-2]], [pts[:,0], pts[:,-2],pts[:,1]]]
        
    # Generating a polygon now
    ax.add_collection3d(Poly3DCollection(verts, facecolors=faceColor,
                                         linewidths=1, edgecolors='k', alpha=.25))