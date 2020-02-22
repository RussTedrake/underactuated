import numpy as np
import matplotlib.pyplot as plt

def plot_2d_phase_portrait(
    f, # rhs of xdot = f(x) where x is 2d array
    x1lim=[-1, 1], # limits for the horizontal axis
    x2lim=[-1, 1], # limits for the vertical axis
    n=100j, # knot points per side
    **kwargs # keyword arguments for streamplot function
):

    # grid state space, careful here x2 before x1
    X2, X1 = np.mgrid[x1lim[0]:x1lim[1]:n, x2lim[0]:x2lim[1]:n]
    X1d, X2d = f([X1, X2])

    # color the streamlines according to the magnitude of xdot
    color = np.sqrt(X1d**2 + X2d**2)
    
    # phase portrait
    strm = plt.streamplot(X1, X2, X1d, X2d, color=color, **kwargs)
    
    # colorbar on the right that measures the magnitude of xdot
    plt.gcf().colorbar(strm.lines, label=r'$|\dot\mathbf{x}|$')
    
    # misc plot settings
    plt.xlabel(r'$x_1$')
    plt.ylabel(r'$x_2$')
    plt.xlim(x1lim)
    plt.ylim(x2lim)
    plt.gca().set_aspect('equal')
