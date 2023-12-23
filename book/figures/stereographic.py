import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse

fig = plt.figure()
ax = plt.gca()

ax.add_patch(
    Ellipse(
        (0, 0), width=2, height=2, edgecolor="k", facecolor="w", linewidth=2
    )
)
ax.axis("equal")
ax.axis(1.15 * np.array([-1, 1, -1, 1]))

theta = np.pi / 4
q = 2 * np.sin(theta) / (1 + np.cos(theta))

ax.plot([-1, 1], [0, 0], color="k", linewidth=0.5)
ax.plot([1, 1], 1.2 * np.array([-1, 1]), color="k", linewidth=0.5)
ax.plot([0, -1, np.cos(theta)], [0, 0, np.sin(theta)], "k.", markersize=10)
ax.plot(
    np.cos(theta) * np.array([1, 1]),
    np.sin(theta) * np.array([0, 1]),
    "k",
    linewidth=0.5,
)
ax.plot(
    np.cos(theta) * np.array([0, 1]),
    np.sin(theta) * np.array([0, 1]),
    "k",
    linewidth=0.5,
)
ax.plot([-1, 1], [0, q], "k", linewidth=0.5)
ax.plot([1.05, 1.2], [q, q], "k", linewidth=0.5)
ax.plot([1.05, 1.2], [0, 0], "k", linewidth=0.5)
ax.annotate(
    "", (1.125, q), xytext=(1.125, 0), arrowprops={"arrowstyle": "<->"}
)
ax.text(0.17, 0.04, r"$\theta$", fontsize=14)
ax.text(1.175, q / 2, r"$2p$", fontsize=14)
ax.plot([0, 0], [-0.05, -0.15], "k", linewidth=0.5)
ax.annotate(
    "", (0, -0.125), xytext=(1, -0.125), arrowprops={"arrowstyle": "<->"}
)
ax.text(0.5, -0.275, "1", fontsize=14)

ax.axis("off")

plt.savefig("book/figures/stereographic.svg")
plt.show()
