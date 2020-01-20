from pydrake.all import MathematicalProgram, Solve

prog = MathematicalProgram()
x = prog.NewIndeterminates(2, "x")
f = [-x[0] - 2*x[1]**2,
     -x[1] - x[0]*x[1] - 2*x[1]**3]

V = x[0]**2 + 2*x[1]**2
Vdot = V.Jacobian(x).dot(f)

prog.AddSosConstraint(-Vdot)

result = Solve(prog)
assert(result.is_success())

print('Successfully verified Lyapunov candidate')
