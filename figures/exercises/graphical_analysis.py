import numpy as np
import matplotlib.pyplot as plt

# change font size to be in line with the textbook
plt.rcParams.update({'font.size': 16})


# piecewise smooth dynamics
def f(x):
    domains = [x <= 1, x <= 2, True]  # first valid is chosen
    dynamics = [-x**5 + 2 * x**3 - x, 0, -x + 2]
    return np.select(domains, dynamics)


# x-axis grid
x_min = -1.3  # chosen so that the 5th power does not explode
x_max = 2.3
knots = 500
x = np.linspace(x_min, x_max, knots)

# plot the graph of f(x)
plt.figure()
plt.plot(x, f(x))
plt.xlim(x_min, x_max)
plt.xlabel(r'$x$')
plt.ylabel(r'$\dot x$')
plt.grid()
plt.savefig('graphical_analysis.svg', bbox_inches='tight')
# plt.show()
