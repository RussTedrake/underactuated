import numpy as np

from pydrake.common.containers import namedview
from pydrake.systems.framework import (BasicVector,
                                       LeafSystem,
                                       PublishEvent,
                                       UnrestrictedUpdateEvent,
                                       WitnessFunctionDirection)

SLIPState = namedview(
    'SLIPState', ['x', 'z', 'r', 'theta', 'xdot', 'zdot', 'rdot', 'thetadot'])


class SpringLoadedInvertedPendulum(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        self.DeclareVectorInputPort("touchdown_angle", BasicVector(1))
        self.DeclareContinuousState(BasicVector(np.zeros(8)), 4, 4, 0)

        self.DeclareVectorOutputPort("state", BasicVector(8),
                                     self.CopyStateOut)

        # Parameters from Geyer05, p.23
        self.mass = 80.  # kg
        self.r0 = 1.  # m
        self.gravity = 9.81  # m/s^2
        # Define spring constant in terms of the dimensionless number.
        # Definition in section 2.4.3, values in figure 2.4.
        # Note: Geyer05 says 10.8 (which doesn't work? -- I get no fixed pts).
        dimensionless_spring_constant = 10.7
        self.stiffness = \
            dimensionless_spring_constant*self.mass*self.gravity/self.r0

        self.last_apex = None  # placeholder for writing return map result.

        self.touchdown_witness = self.MakeWitnessFunction(
            "touchdown", WitnessFunctionDirection.kPositiveThenNonPositive,
            self.foot_height, UnrestrictedUpdateEvent(self.touchdown))
        self.takeoff_witness = self.MakeWitnessFunction(
            "takeoff", WitnessFunctionDirection.kPositiveThenNonPositive,
            self.leg_compression, UnrestrictedUpdateEvent(self.takeoff))
        self.apex_witness = self.MakeWitnessFunction(
            "apex", WitnessFunctionDirection.kPositiveThenNonPositive,
            self.apex, PublishEvent(self.publish_apex))

    def foot_height(self, context):
        s = SLIPState(context.get_continuous_state_vector().CopyToVector())
        return s.z - self.r0*np.cos(s.theta)

    def touchdown(self, context, event, state):
        s = SLIPState(
            context.get_mutable_continuous_state_vector().CopyToVector())

        # print("touchdown")

        # Update rdot and thetadot to match xdot and ydot, using
        # x = -r*sin(theta), z = r*cos(theta)
        #  => xdot = -rdot*s - r*c*thetadot, zdot = rdot*c - r*s*thetadot
        #  => xdot*c + zdot*s = -r*thetadot
        # r^2 = x^2 + z^2
        #  => 2r*rdot = 2x*xdot + 2z*zdot
        #  => rdot = -xdot*sin(theta) + zdot*cos(theta)
        # (matches Geyer05 Eq. 2.24 up to the symbol changes)
        s.r = self.r0
        s.rdot = -s.xdot*np.sin(s.theta) + s.zdot*np.cos(s.theta)
        s.thetadot = -(s.xdot*np.cos(s.theta) + s.zdot*np.sin(s.theta))/self.r0
        state.get_mutable_continuous_state().get_mutable_vector()\
            .SetFromVector(s[:])

    def leg_compression(self, context):
        s = SLIPState(context.get_continuous_state_vector().CopyToVector())
        return self.r0 - s.r

    def takeoff(self, context, event, state):
        s = SLIPState(
            context.get_mutable_continuous_state_vector().CopyToVector())

        # print("takeoff")

        # Setup flight state (these lines aren't strictly required, since we
        # choose to also integrate x and z in stance below).
        s.z = self.r0*np.cos(s.theta)
        s.xdot = -s.rdot*np.sin(s.theta) - self.r0*s.thetadot*np.cos(s.theta)
        s.zdot = s.rdot*np.cos(s.theta) - self.r0*s.thetadot*np.sin(s.theta)

        # Update theta to commanded leg angle.
        s.theta = self.EvalVectorInput(context, 0).GetAtIndex(0)
        s.thetadot = 0
        s.r = self.r0
        s.rdot = 0

        state.get_mutable_continuous_state().get_mutable_vector()\
            .SetFromVector(s[:])

    def apex(self, context):
        return context.get_continuous_state_vector().GetAtIndex(5)  # zdot

    def publish_apex(self, context, event):
        # TODO(russt): provide an option to terminate here instead, pending
        # resolution of #4447.
        # print("apex")
        if self.last_apex is None:
            s = SLIPState(
                context.get_mutable_continuous_state_vector().CopyToVector())
            self.last_apex = s.z

    def apex_velocity_from_dimensionless_system_energy(self, Etilde, z):
        E = Etilde*self.mass*self.gravity*self.r0
        # E = 0.5*m*v^2 + m*g*z
        xdot = np.sqrt(2./self.mass*(E - self.mass*self.gravity*z))
        return xdot

    def energy_flight(self, context):
        s = SLIPState(
            context.get_mutable_continuous_state_vector().CopyToVector())
        return 0.5*self.mass*(s.xdot**2 + s.zdot**2) \
            + self.mass*self.gravity*s.z

    def energy_stance(self, context):
        s = SLIPState(
            context.get_mutable_continuous_state_vector().CopyToVector())
        return 0.5*self.mass*(s.rdot**2 + s.r**2*s.thetadot**2) \
            + self.mass*self.gravity*s.r*np.cos(s.theta) \
            + 0.5*self.stiffness*(self.r0 - s.r)**2

    def CopyStateOut(self, context, output):
        x = context.get_continuous_state_vector().CopyToVector()
        y = output.SetFromVector(x)

    def DoGetWitnessFunctions(self, context):
        return [self.touchdown_witness, self.takeoff_witness,
                self.apex_witness]

    def DoCalcTimeDerivatives(self, context, derivatives):
        s = SLIPState(context.get_continuous_state_vector().CopyToVector())
        sdot = SLIPState(np.zeros(8))
        sdot[0:4] = s[4:8]

        if (self.foot_height(context) < 0):
            # then we're in "stance"
            sdot.rdot = self.stiffness/self.mass*(self.r0-s.r) \
                        + s.r*s.thetadot**2 \
                        - self.gravity*np.cos(s.theta)
            sdot.thetadot = self.gravity/s.r*np.sin(s.theta) \
                            - 2*s.rdot*s.thetadot/s.r  # noqa

            # Integrate x and z also, just for the sake of visualization (all
            # the integrated values except x will be overwritten in the
            # take-off reset).
            # x = -r*sin(theta), y = r*cos(theta) =>
            sdot.xdot = - sdot.rdot*np.sin(s.theta) \
                        - 2*s.rdot*s.thetadot*np.cos(s.theta) \
                        + s.r*s.thetadot**2*np.sin(s.theta) \
                        - s.r*sdot.thetadot*np.cos(s.theta)  # noqa
            sdot.zdot = sdot.rdot*np.cos(s.theta) \
                            - 2*s.rdot*s.thetadot*np.sin(s.theta) \
                            - s.r*sdot.thetadot*np.sin(s.theta) \
                            - s.r*s.thetadot**2*np.cos(s.theta)  # noqa

        else:
            sdot.xdot = 0
            sdot.zdot = -self.gravity
            sdot.rdot = 0
            sdot.thetadot = 0

        derivatives.get_mutable_vector().SetFromVector(sdot[:])
