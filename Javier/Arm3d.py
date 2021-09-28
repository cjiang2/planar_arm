from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

def main():
    print('Hello world')
    fig1 = plt.figure()
    ax1 = plt.axes(projection='3d')
    #ax1.axis('equal')
    plot_link([0,0,0], [1,2,3], ax1)
    plt.show()

def plot_link(_to, _from, ax):
    arr_pts = np.array([_to] + [_from])
    print(arr_pts)
    x = arr_pts[:, 0]
    y = arr_pts[:, 1]
    z = arr_pts[:, 2]
    ax.plot3D(x, y, z)


if __name__ == '__main__':
    main()
