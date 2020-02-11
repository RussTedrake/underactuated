import matplotlib.pyplot as plt
import numpy as np

def sigma(x):
  return 1./(1+np.exp(-x))

def lstm(x, uf=0, ui=0, u=0):
  af = 1
  bf = 1
  cf = 0
  ai = 1
  bi = 1
  ci = 0
  b = 1
  c = 0
  return - x + sigma(af*x + bf*uf + cf)*x + sigma(ai*x + bi*ui + c) * np.tanh(b*u+c)


Lstm = np.vectorize(lstm)
xmax = 10.
ymax = 4.
xs = np.arange(-xmax, xmax, 0.01)

def plot_lstm(uf=0, ui=0, u=0):
  plt.plot(xs, Lstm(xs, uf=uf, ui=ui, u=u), linewidth=2., label="uf=" + str(uf) + ",ui=" + str(ui) + ", u=" + str(u))

#plot_lstm(uf=-10, ui=-10)

#for us in range(-5,5,2):
#  plot_lstm(uf=us, ui=-10)

for us in np.arange(-1,1,.25):
  plot_lstm(uf=0, ui=0, u=us)

plt.xlabel("x")
plt.ylim((-ymax, ymax))
plt.ylabel("xdot")
plt.legend()

# draw the x and y axes.
plt.plot([-xmax, xmax], [0, 0], color="k", linestyle="-", linewidth=1.)
plt.plot([0, 0], [-ymax, ymax], color="k", linestyle="-", linewidth=1.)
# draw the line through the origin with slope -1.
plt.plot([-ymax, ymax], [ymax, -ymax], color="k", linestyle="-", linewidth=1.)

plt.title("LSTM")
plt.axis("equal")
plt.show()
