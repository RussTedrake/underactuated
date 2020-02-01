import numpy as np
import matplotlib.pyplot as plt

# change font size to be in line with the textbook
plt.rcParams.update({'font.size': 16})


# dynamics of the double integrator
def f(x1, x2):
    return [x2, np.zeros(x2.shape)]


# grid state space for phase portrait
dx = 5.
dy = 1.
n = 11j
X2, X1 = np.mgrid[-dy:dy:n, -dx:dx:n]

# phase portrait
X1d, X2d = f(X1, X2)
strm = plt.quiver(
    X1, X2, X1d, X2d, angles='xy', scale_units='xy',
    scale=1)  # options to make the scaling of xdot the same as the one of x

# initial states as dots and trajecories as arrows
initial_states = [(np.array([2, .5]), 'b'), (np.array([2, -.5]), 'r')]
for x, c in initial_states:
    plt.scatter(*x,
                c=c,
                label=r'$\mathbf{x}(0)=[%s,%s]^T$' % (str(x[0]), str(x[1])))
    plt.arrow(*x,
              *-x,
              color=c,
              width=.01,
              head_width=.05,
              head_length=.2,
              length_includes_head=True)

# misc plot settings
plt.xlabel(r'$q$')
plt.ylabel(r'$\dot q$')
plt.xlim(-dx, dx)
plt.ylim(-dy, dy)
plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.2), ncol=2)
plt.savefig('trajectory_tracking.svg', bbox_inches='tight')
# plt.show()
