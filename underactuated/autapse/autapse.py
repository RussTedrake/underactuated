import matplotlib.pyplot as plt
import numpy as np


def autapse(x, w=1, u=0):
    """Args:
    w is feedback weight
    u is input
    """
    return -x + np.tanh(w * x + u)


Autapse = np.vectorize(autapse)
xmax = 2.
ymax = 1.
t = np.arange(-xmax, xmax, 0.01)

#plt.plot(t, Autapse(t, w=.75, u=0), linewidth=2., label="w=3/4")
#plt.plot(t, Autapse(t, w=3., u=0), linewidth=2., label="w=3")

for u in np.linspace(-1,1,5):
  plt.plot(t, Autapse(t, w=2., u=u), linewidth=2., label="w=2, u="+str(u))

plt.xlabel("x")
plt.ylim((-ymax, ymax))
plt.ylabel("xdot")
plt.legend()

# draw the x and y axes.
plt.plot([-xmax, xmax], [0, 0], color="k", linestyle="-", linewidth=1.)
plt.plot([0, 0], [-ymax, ymax], color="k", linestyle="-", linewidth=1.)
# draw the line through the origin with slope -1.
plt.plot([-ymax, ymax], [ymax, -ymax], color="k", linestyle="-", linewidth=1.)

plt.title("Autapse")
plt.axis("equal")
plt.show()
