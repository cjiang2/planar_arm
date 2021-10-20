import numpy as np
import matplotlib.pyplot as plt

from arm.arm3d import Arm3D
from arm.camera import PinholeCam, CCDCam, OrthoProjCam

if __name__ == '__main__':
    # Start with a random angle
    t = np.random.uniform(size=(3, 1))
    
    # Init the camera as well
    # Rotation: Z points to robot, Y straight upwards, X perpendicular
    pcam1 = PinholeCam(f=5.0, 
                       px=3.0, 
                       py=3.0, 
                       C=np.array([1.5, 1.5, 0.25]),
                       theta=np.array([np.pi/2, np.pi / 2 + np.pi/4, 0]))

    ccdcam1 = PinholeCam(f=5.0, 
                     px=3.0,
                     py=3.0,
                     C=np.array([2.0, -0.5, 0.25]),
                     theta=np.array([np.pi/2, 3*np.pi / 8, 0]))

    opcam1 = OrthoProjCam()
    
    arm_env = Arm3D(t, 
                    cams=[pcam1, ccdcam1])
    arm_env.plot()

    
    