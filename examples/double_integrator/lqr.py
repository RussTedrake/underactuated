import numpy as np
from pydrake.systems.controllers import LinearQuadraticRegulator

# Define the double integrator's state space matrices.
A = np.array([[0, 1], [0, 0]])
B = np.array([[0], [1]])
Q = np.eye(2)
R = np.eye(1)

(K, S) = LinearQuadraticRegulator(A, B, Q, R)
print("K = " + str(K))
print("S = " + str(S))
