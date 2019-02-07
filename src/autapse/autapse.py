import matplotlib.pyplot as plt
import numpy as np


def autapse(x, w=1, u=0, f=1):
    '''
    w is feedback weight
    u is input
    f is the "forget gate" (f=1 => no forgetting)
    '''
    return -f*x + np.tanh(w*x + u)


Autapse = np.vectorize(autapse)
xmax = 2.
ymax = 2.
t = np.arange(-xmax, xmax, 0.01)

plt.plot(t, Autapse(t, w=2., u=0), linewidth=2.)
plt.xlabel('x')
plt.ylim((-ymax, ymax))
plt.ylabel('xdot')

plt.plot([-xmax, xmax], [0, 0], color='k', linestyle='-', linewidth=1.)
plt.plot([0, 0], [-ymax, ymax], color='k', linestyle='-', linewidth=1.)
plt.title('Autapse')
plt.show()
