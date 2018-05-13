import math

from pydrake.all import (Jacobian, MathematicalProgram, SolutionResult,
                         Variables)


def dynamics(x):
    return -x + x**3


prog = MathematicalProgram()
x = prog.NewIndeterminates(1, "x")
rho = prog.NewContinuousVariables(1, "rho")[0]

# Define the Lyapunov function.
V = x.dot(x)
Vdot = Jacobian([V], x).dot(dynamics(x))[0]

# Define the Lagrange multipliers.
(lambda_, constraint) = prog.NewSosPolynomial(Variables(x), 4)

prog.AddSosConstraint((V-rho) * x.dot(x) - lambda_.ToExpression() * Vdot)
prog.AddLinearCost(-rho)

result = prog.Solve()

assert(result == SolutionResult.kSolutionFound)

print("Verified that " + str(V) + " < " + str(prog.GetSolution(rho)) +
      " is in the region of attraction.")

assert(math.fabs(prog.GetSolution(rho) - 1) < 1e-5)
