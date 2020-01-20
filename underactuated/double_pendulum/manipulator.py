from pydrake.all import MultibodyPlant, Parser, Variable
from underactuated import FindResource, ManipulatorDynamics

plant = MultibodyPlant(time_step=0)
parser = Parser(plant)
parser.AddModelFromFile(FindResource("double_pendulum/double_pendulum.urdf"))
plant.Finalize()

# Evaluate the dynamics numerically
q = [0.1, 0.1]
v = [1, 1]
(M, Cv, tauG, B, tauExt) = ManipulatorDynamics(plant, q, v)
print("M = \n" + str(M))
print("Cv = " + str(Cv))
print("tau_G = " + str(tauG))
print("B = " + str(B))
print("tau_ext = " + str(tauExt))

# Evaluate the dynamics symbolically
q = [Variable("theta0"), Variable("theta1")]
v = [Variable("thetadot0"), Variable("thetadot1")]
(M, Cv, tauG, B, tauExt) = ManipulatorDynamics(plant.ToSymbolic(), q, v)
print("M = \n" + str(M))
print("Cv = " + str(Cv))
print("tau_G = " + str(tauG))
print("B = " + str(B))
print("tau_ext = " + str(tauExt))
