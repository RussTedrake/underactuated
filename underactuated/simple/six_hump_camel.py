import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from pydrake.all import MathematicalProgram, Solve

prog = MathematicalProgram()
v = prog.NewIndeterminates(2, "x")
x = v[0]
y = v[1]

# This is the famous "six-hump camel back function".  It has six local
# minima, two of them being global minima.
p = 4*x**2+x*y - 4*y**2 - 2.1*x**4 + 4*y**4+x**6/3

# Find the minimum value by adding a sums of squares constraint, via
#   for all x, p(x) >= pmin
# which we write as
#   p(x) - pmin is sos.
pmin = prog.NewContinuousVariables(1, "pmin")[0]
prog.AddSosConstraint(p-pmin)

# Maximize pmin.
prog.AddCost(-pmin)

result = Solve(prog)
assert(result.is_success())
print("Minimum value (lower bound): " + str(result.GetSolution(pmin)))


# Now, let's plot it.
fig = plt.figure(figsize=(10, 5))
ax0 = fig.add_subplot(121, projection='3d')
ax1 = fig.add_subplot(122)
xs = np.linspace(-2.2, 2.2, 51)
ys = np.linspace(-1.2, 1.2, 51)
[X, Y] = np.meshgrid(xs, ys)
P = X.copy()
for i in range(len(xs)):
    for j in range(len(ys)):
        P[i, j] = p.Evaluate({x: X[i, j], y: Y[i, j]})
ax0.plot_surface(X, Y, P)
ax1.contour(X, Y, P, 100)

print("Minimum sampled value: " + str(np.min(P)))

plt.show()
