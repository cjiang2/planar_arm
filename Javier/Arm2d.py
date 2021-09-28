import numpy as np
from math import sin, cos
import matplotlib.pyplot as plt

def main():
    print('2D robot')
    a = [3, 2, 1]
    theta = [np.pi/3, -np.pi/8, -np.pi/10]

    #1. Homogenous transformations
    positions = fk(a, theta) # Hardcoded for 3 links, could be generalized. But save that for 3d
    xy1 = positions[0] 
    xy2 = positions[1]
    xy3 = positions[2]
    xy4 = positions[3]
    print((xy2[1]-xy1[1])/(xy2[0]-xy1[0]))
    print((xy3[1]-xy2[1])/(xy3[0]-xy2[0]))
    print((xy4[1]-xy3[1])/(xy4[0]-xy3[0]))

    fig1 = plt.figure()
    ax1 = plt.gca()
    ax1.axis('equal')
    plot_link(      [0,0], xy1, ax1)
    plot_link(xy1, xy2, ax1)
    plot_link(xy2, xy3, ax1)
    plot_link(xy3, xy4, ax1)
    plt.show()
    
    # Analytical inverse kinematics

def get_xy(xya):
    return [xya[0,0], xya[1,0]]

def fk(a, theta):
    k = len(a)
    T1 = np.asmatrix([[cos(theta[0]), -sin(theta[0]), 0],
                      [sin(theta[0]),  cos(theta[0]), 0],
                      [            0,              0, 1]])

    T2 = np.asmatrix([[cos(theta[1]), -sin(theta[1]), a[0]],
                      [sin(theta[1]),  cos(theta[1]),    0],
                      [            0,              0,    1]])
                      
    T3 = np.asmatrix([[cos(theta[2]), -sin(theta[2]), a[1]],
                      [sin(theta[2]),  cos(theta[2]),    0],
                      [            0,              0,    1]])

    T4 = np.asmatrix([[            1,              0, a[2]],
                      [            0,              1,     0],
                      [            0,              0,     1]])

    T0 = np.asmatrix([[0],[0],[1]])
    
    xy1 = T1 * T0
    xy2 = T1 * T2 * T0 
    xy3 = T1 * T2 * T3 * T0 
    xy4 = T1 * T2 * T3 * T4 * T0
    return [get_xy(xy1), get_xy(xy2), get_xy(xy3), get_xy(xy4)]
    
def plot_link(_from, _to, ax):
    arr_pts = np.array([_from] + [_to])
    x = arr_pts[:, 0]
    y = arr_pts[:, 1]
    ax.plot(x, y, color = 'black')
    circ1 = plt.Circle(_to, 0.05, color = 'black', fill = False)
    circ2 = plt.Circle(_from, 0.1, color = 'black', fill = False)
    ax.add_artist(circ1)
    ax.add_artist(circ2)

if __name__ == '__main__':
    main()
