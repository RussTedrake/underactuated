import numpy as np

from pydrake.common.containers import namedview
from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.geometry import FramePoseVector
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import (BasicVector, BasicVector_, LeafSystem_,
                                       LeafSystem)
from pydrake.systems.scalar_conversion import TemplateSystem

from underactuated import FindResource

# Note: In order to use the Python system with drake's autodiff features, we
# have to add a little "TemplateSystem" boilerplate (for now).  For details,
# see https://drake.mit.edu/pydrake/pydrake.systems.scalar_conversion.html

GliderState = namedview(
    "GliderState", ["x", "z", "pitch", "elevator", "xdot", "zdot", "pitchdot"])


# TODO(russt): Clean this up pending any resolutions on
#  https://github.com/RobotLocomotion/drake/issues/10745
@TemplateSystem.define("GliderPlant_")
def GliderPlant_(T):

    class Impl(LeafSystem_[T]):

        def _construct(self, converter=None):
            LeafSystem_[T].__init__(self, converter)
            # one inputs (elevator_velocity)
            self.DeclareVectorInputPort("elevatordot", BasicVector_[T](1))
            # four positions, three velocities
            self.DeclareContinuousState(4, 3, 0)
            # seven outputs (full state)
            self.DeclareVectorOutputPort("state", BasicVector_[T](7),
                                         self.CopyStateOut)

            # TODO(russt): Declare elevator constraints:
            # elevator_lower_limit = -0.9473
            # elevator_upper_limit = 0.4463

        def _construct_copy(self, other, converter=None):
            Impl._construct(self, converter=converter)

        def DoCalcTimeDerivatives(self, context, derivatives):
            # parameters based on Rick Cory's "R1 = no dihedral" model.
            Sw = 0.0885  # surface area of wing + fuselage + tail.
            Se = 0.0147  # surface area of elevator.
            lw = 0  # horizontal offset of wing center.
            le = 0.022  # elevator aerodynamic center from hinge.
            lh = 0.317  # elevator hinge.
            inertia = 0.0015  # body inertia.
            m = 0.08  # body mass.
            rho = 1.204  # air density (kg/m^3).
            gravity = 9.81  # gravity

            s = GliderState(
                context.get_mutable_continuous_state_vector().CopyToVector())
            elevatordot = self.EvalVectorInput(context, 0).CopyToVector()

            xwdot = s.xdot + lw * s.pitchdot * np.sin(s.pitch)
            zwdot = s.zdot + lw * s.pitchdot * np.cos(s.pitch)
            alpha_w = -np.atan2(zwdot, xwdot) - s.pitch
            fw = rho * Sw * np.sin(alpha_w) * (zwdot**2 + xwdot**2)

            e = s.pitch + s.elevator
            edot = s.pitchdot + elevatordot
            xedot = s.xdot + lh * s.pitchdot * np.sin(s.pitch) \
                + le * edot * np.sin(e)
            zedot = s.zdot + lh * s.pitchdot * np.cos(s.pitch) \
                + le * edot * np.cos(e)
            alpha_e = -atan2(zedot, xedot) - e
            fe = rho * Se * np.sin(alpha_e) * (zedot**2 + xedot**2)

            xdot = [
                xdot, zdot, pitchdot, elevatordot,
                (fw * np.sin(s.pitch) + fe * np.sin(e)) / m,
                (fw * np.cos(s.pitch) + fe * np.cos(e)) / m - gravity,
                (fw * lw + fe * (lh * np.cos(s.elevator) + le)) / inertia
            ]
            derivatives.get_mutable_vector().SetFromVector(xdot)

        def CopyStateOut(self, context, output):
            x = context.get_continuous_state_vector().CopyToVector()
            y = output.SetFromVector(x)

    return Impl


# To use glider.urdf for visualization, follow the pattern from e.g.
# drake::examples::quadrotor::QuadrotorGeometry.
class GliderGeometry(LeafSystem):

    def __init__(self, scene_graph):
        LeafSystem.__init__(self)
        assert scene_graph

        mbp = MultibodyPlant(0.0)
        parser = Parser(mbp, scene_graph)
        model_id = parser.AddModelFromFile(
            FindResource("models/glider/glider.urdf"))
        mbp.Finalize()
        self.source_id = mbp.get_source_id()
        self.body_frame_id = mbp.GetBodyFrameIdOrThrow(
            mbp.GetBodyIndices(model_id)[0])
        self.elevator_frame_id = mbp.GetBodyFrameIdOrThrow(
            mbp.GetBodyIndices(model_id)[1])

        self.DeclareVectorInputPort("state", BasicVector(7))
        self.DeclareAbstractOutputPort(
            "geometry_pose", lambda: AbstractValue.Make(FramePoseVector()),
            self.OutputGeometryPose)

    def OutputGeometryPose(self, context, poses):
        assert self.body_frame_id.is_valid()
        assert self.elevator_frame_id.is_valid()
        lh = 0.317  # elevator hinge.
        state = GliderState(self.get_input_port(0).Eval(context))
        body_pose = RigidTransform(RotationMatrix.MakeYRotation(state.pitch),
                                   [state.x, 0, state.z])
        elevator_pose = RigidTransform(
            RotationMatrix.MakeYRotation(state.pitch + state.elevator), [
                state.x - lh * np.cos(state.pitch), 0,
                state.z + lh * np.sin(state.pitch)
            ])
        poses.get_mutable_value().set_value(self.body_frame_id, body_pose)
        poses.get_mutable_value().set_value(self.elevator_frame_id,
                                            elevator_pose)

    @staticmethod
    def AddToBuilder(builder, glider_state_port, scene_graph):
        assert builder
        assert scene_graph

        geom = builder.AddSystem(GliderGeometry(scene_graph))
        builder.Connect(glider_state_port, geom.get_input_port(0))
        builder.Connect(geom.get_output_port(0),
                        scene_graph.get_source_pose_port(geom.source_id))

        return geom


GliderPlant = GliderPlant_[None]  # Default instantiation
