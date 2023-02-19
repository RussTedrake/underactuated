from dreal import *  # pip3 install dreal.
x = Variable("x")

# To use dReal, we flip the inequality and verify that the slope being less
# that mu is unsatisfiable (over the finite domain).  I then found the largest
# mu that returns "none" (== unsatisfiable).  Larger values provide a witness
# that can verify satisfiability.
mu = .175
result = CheckSatisfiability(And(
    Or(And(-1e6 <= x, x <= -1e-6), And(1e-6 <= x, x <= 1e6)),
    0.5*(2*x + 6*sin(x)*cos(x))**2 <= mu*(x**2 + 3*sin(x)**2)
), 1e-6)
print(result)
