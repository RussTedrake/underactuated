from pydrake.all import MultibodyPlant, Parser
from underactuated import FindResource, ManipulatorDynamics

plant = MultibodyPlant()
parser = Parser(plant)
parser.AddModelFromFile(FindResource("double_pendulum/double_pendulum.urdf"))
plant.Finalize()

q = [0.1, 0.1]
v = [1, 1]
(M, Cv, tauG, B, tauExt) = ManipulatorDynamics(plant, q, v)

print("M = \n" + str(M))
print("Cv = " + str(Cv))
print("tau_G = " + str(tauG))
print("B = " + str(B))
print("tau_ext = " + str(tauExt))

# TODO(russt): add symbolic version pending resolution of drake #11240
