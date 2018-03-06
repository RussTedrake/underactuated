import numpy as np
from pydrake.all import (MathematicalProgram, SolutionResult)

A = []
if (True):
    # Generate random stable matrices.
    num_states = 4
    num_systems = 2
    for i in range(num_systems):
        d = -np.random.rand(num_states,)
        v = np.random.randn(num_states, num_states)
        A.append(v.dot(np.diag(d).dot(np.linalg.inv(v))))
else:
    # Example from lecture notes.
    A.append = np.array(((-1, .5), (-3, -1)))
    A.append = np.array(((-1, .1), (-10, -1)))
    # Interesting for 2D plotting (a two element parameterization of stable
    # linear systems).  Stable iff ab < 1.
    # a = randn;  ab = 2*rand - 1;  b=ab/a;
    # A{i} = [-1 a; b -1];


# Create the optimization problem.
prog = MathematicalProgram()

# Construct an n-by-n positive semi-definite matrix as the decision
# variables.
num_states = A[0].shape[0]
P = prog.NewSymmetricContinuousVariables(num_states, "P")
prog.AddPositiveSemidefiniteConstraint(P - .01*np.identity(num_states))

# Add the common Lyapunov conditions.
for i in range(len(A)):
    prog.AddPositiveSemidefiniteConstraint(-A[i].transpose().dot(P)
                                           - P.dot(A[i])
                                           - .01*np.identity(num_states))

# Add an objective.
prog.AddLinearCost(np.trace(P))

# Run the optimization.
result = prog.Solve()

if result == SolutionResult.kSolutionFound:
    P = prog.GetSolution(P)
    print("eig(P) =" + str(np.linalg.eig(P)[0]))
    for i in range(len(A)):
        print("eig(Pdot" + str(i) + ") = " +
              str(np.linalg.eig(A[i].transpose().dot(P) + P.dot(A[i]))[0]))
else:
    print('Could not find a common Lyapunov function.')
    print('This is expected to occur with some probability:  not all')
    print('random sets of stable matrices will have a common Lyapunov')
    print('function.')
