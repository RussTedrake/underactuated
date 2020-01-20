from pydrake.all import MathematicalProgram, Solve, Polynomial, Variables

prog = MathematicalProgram()
x = prog.NewIndeterminates(2, "x")
f = [-x[0] - 2*x[1]**2,
     -x[1] - x[0]*x[1] - 2*x[1]**3]

V = prog.NewSosPolynomial(Variables(x), 2)[0].ToExpression()
prog.AddLinearConstraint(V.Substitute({x[0]: 0, x[1]: 0}) == 0)
prog.AddLinearConstraint(V.Substitute({x[0]: 1, x[1]: 0}) == 1)
Vdot = V.Jacobian(x).dot(f)

prog.AddSosConstraint(-Vdot)

result = Solve(prog)
assert(result.is_success())

print('V = ' + str(Polynomial(result.GetSolution(V))
                   .RemoveTermsWithSmallCoefficients(1e-5)))
