import math
import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (MathematicalProgram, SolutionResult, Variables)


def dynamics(x):
    return -x + x**3


prog = MathematicalProgram()
x = prog.NewIndeterminates(1, "x")

# Define the Lyapunov function.
V = x.dot(x)

# Short circuit until I have symbolic::Polynomial::Jacobian
# Vdot = V.Jacobian(x).dot(dynamics(x))
Vdot = 2.*x.dot(dynamics(x))
rho = prog.NewContinuousVariables(1, "rho")[0]
(lambda_, constraint) = prog.NewSosPolynomial(Variables(x), 4)

prog.AddSosConstraint((V-rho) * x.dot(x) - lambda_.ToExpression() * Vdot)
prog.AddLinearCost(-rho)

result = prog.Solve()

assert(result == SolutionResult.kSolutionFound)

print("Verified that " + str(V) + " < " + str(prog.GetSolution(rho)) +
      " is in the region of attraction.")

assert(math.fabs(prog.GetSolution(rho) - 1) < 1e-5)
