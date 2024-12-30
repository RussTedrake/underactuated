"""Defines the UAV environment."""

from enum import Enum
from typing import Dict, List, Optional, Sequence, Set, Tuple

# Define the IBM color map
import matplotlib.colors as mcolors
import numpy as np
from lxml.etree import Element, ElementTree, SubElement, tostring
from pydrake.common.value import Value
from pydrake.geometry import FramePoseVector, Meshcat, Rgba, Role, SceneGraph
from pydrake.geometry.optimization import (
    GraphOfConvexSetsOptions,
    HPolyhedron,
    VPolytope,
)
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.solvers import (
    ClarabelSolver,
    GurobiSolver,
    MosekSolver,
    SnoptSolver,
    SolverOptions,
)
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import (
    BasicVector,
    Context,
    DiagramBuilder,
    EventStatus,
    InputPort,
    LeafSystem,
)
from pydrake.systems.primitives import TrajectorySource
from pydrake.trajectories import BezierCurve, CompositeTrajectory, Trajectory
from pydrake.visualization import ApplyVisualizationConfig, VisualizationConfig
from scipy.spatial import ConvexHull

from underactuated import ConfigureParser

colors = ["#648FFF", "#DC267F", "#FE6100", "#FFB000"]  # "#785EF0"]
cmap_ibm = mcolors.LinearSegmentedColormap.from_list("IBM", colors)

CONVEX_GCS_OPTION = GraphOfConvexSetsOptions()
CONVEX_GCS_OPTION.solver = (
    MosekSolver()
    if MosekSolver().available() and MosekSolver().enabled()
    else ClarabelSolver()
)
CONVEX_GCS_OPTION.solver_options = SolverOptions()
CONVEX_GCS_OPTION.solver_options.SetOption(
    MosekSolver.id(), "MSK_DPAR_INTPNT_CO_TOL_REL_GAP", 1e-3
)
CONVEX_GCS_OPTION.solver_options.SetOption(
    MosekSolver.id(), "MSK_IPAR_INTPNT_SOLVE_FORM", 1
)
CONVEX_GCS_OPTION.solver_options.SetOption(
    MosekSolver.id(), "MSK_DPAR_MIO_TOL_REL_GAP", 1e-3
)
CONVEX_GCS_OPTION.solver_options.SetOption(
    MosekSolver.id(), "MSK_DPAR_MIO_MAX_TIME", 3600.0
)
CONVEX_GCS_OPTION.solver_options.SetOption(GurobiSolver.id(), "MIPGap", 1e-3)
CONVEX_GCS_OPTION.solver_options.SetOption(GurobiSolver.id(), "TimeLimit", 3600.0)
CONVEX_GCS_OPTION.max_rounded_paths = 12
CONVEX_GCS_OPTION.preprocessing = True
# CONVEX_GCS_OPTION.parallelize = Parallelism(CONVEX_GCS_OPTION.max_rounded_paths)

NONLINEAR_GCS_OPTION = GraphOfConvexSetsOptions()
NONLINEAR_GCS_OPTION.max_rounded_paths = 12
NONLINEAR_GCS_OPTION.solver = CONVEX_GCS_OPTION.solver
NONLINEAR_GCS_OPTION.solver_options = CONVEX_GCS_OPTION.solver_options
NONLINEAR_GCS_OPTION.restriction_solver = SnoptSolver()
restriction_solver_options = SolverOptions()
restriction_solver_options.SetOption(SnoptSolver.id(), "Iterations Limits", 1e5)
restriction_solver_options.SetOption(
    SnoptSolver.id(), "Major Feasibility Tolerance", 1e-4
)
restriction_solver_options.SetOption(
    SnoptSolver.id(), "Minor Feasibility Tolerance", 1e-5
)
restriction_solver_options.SetOption(SnoptSolver.id(), "Function Precision", 1e-9)
restriction_solver_options.SetOption(
    SnoptSolver.id(), "Major Optimality Tolerance", 1e-4
)
restriction_solver_options.SetOption(
    SnoptSolver.id(), "Minor Optimality Tolerance", 1e-5
)
restriction_solver_options.SetOption(SnoptSolver.id(), "Major Iterations Limit", 1000)
restriction_solver_options.SetOption(SnoptSolver.id(), "Superbasics limit", 209)
NONLINEAR_GCS_OPTION.restriction_solver_options = restriction_solver_options
NONLINEAR_GCS_OPTION.preprocessing = True
# NONLINEAR_GCS_OPTION.use_initial_guess = False
# NONLINEAR_GCS_OPTION.parallelize = Parallelism(NONLINEAR_GCS_OPTION.max_rounded_paths)


# The following constants are used to define the environment.
# The environment is a grid of cells, each of which is a square of size
# CELL_SIZE meters.
CELL_SIZE = 5.0  # meters.
# The building is of size CELL_SIZE x CELL_SIZE x BUILDING_HEIGHT meters.
BUILDING_HEIGHT = 3.0  # meters.
# All walls are WALL_THICKNESS meters thick.
WALL_THICKNESS = 0.25  # meters.

# The quad copter is QUAD_COPTER_DIAMETER meters in diameter.
# This is used to shrink the save collision free space in the building, so that
# some margin is left to consider the quadcopter's size.
QUAD_COPTER_DIAMETER = 0.4  # meters.


ASSET_WIDTH = 1  # meters.


def shift_composite_trajectory(
    traj: CompositeTrajectory, offset_time: float
) -> CompositeTrajectory:
    """Shifts a composite trajectory by a given offset time.

    Args:
        traj: A composite trajectory consisting of bezier curves.
        offset_time: The amount of time to shift the trajectory by.

    Returns:
        A composite trajectory shifted by offset_time.
    """
    bezier_curves: List[BezierCurve] = []
    for i in range(traj.get_number_of_segments()):
        traj_segment = traj.segment(i)
        if not isinstance(traj_segment, BezierCurve):
            raise ValueError("Trajectory is not a composite of bezier curves.")

        bezier_curves.append(
            BezierCurve(
                traj_segment.start_time() + offset_time,
                traj_segment.end_time() + offset_time,
                traj_segment.control_points(),
            )
        )
    return CompositeTrajectory(bezier_curves)  # type: ignore


class TraceVisualizer(LeafSystem):
    """A meshcat visualizer for cartesian positions."""

    # The tolerance for checking if the position has changed.
    POSITION_TOLERANCE = 1e-2

    def __init__(
        self,
        meshcat: Meshcat,
        line_width: float = 1.0,
        rgba: Rgba = Rgba(0.5, 0.5, 0.5, 0.5),
    ) -> None:
        """Trace a given trajectory input.

        Input_ports:
            position: The cartesian position to trace (x, y, z).

        Args:
            meshcat: The meshcat instance to use.
            line_width: The width of the trace line.
            rgba: The color of the trace line.
        """
        super().__init__()
        self._meshcat = meshcat
        self._line_width = line_width
        self._trace_color = rgba

        self._input_port = self.DeclareVectorInputPort("position", 3)
        self.DeclarePerStepPublishEvent(self.visualize_trace)

        self._last_position = np.zeros(3)
        self._position_cnt = 0

    def visualize_trace(self, context: Context) -> EventStatus:
        """Visualize the trace of the input position."""
        position = self._input_port.Eval(context)

        if self._position_cnt:
            if not np.allclose(
                position, self._last_position, atol=self.POSITION_TOLERANCE
            ):
                self._meshcat.SetLine(
                    path=f"traces/{self.get_name()}/{self._position_cnt}",
                    vertices=np.array([self._last_position, position]).T,
                    line_width=self._line_width,
                    rgba=self._trace_color,
                )

                # Show the trace at its corresponding time.
                self._meshcat.SetProperty(
                    path=f"traces/{self.get_name()}/{self._position_cnt}",
                    property="visible",
                    value=False,
                    time_in_recording=0.0,
                )

                self._meshcat.SetProperty(
                    path=f"traces/{self.get_name()}/{self._position_cnt}",
                    property="visible",
                    value=True,
                    time_in_recording=context.get_time(),
                )

                self._last_position = position
                self._position_cnt += 1
        else:
            self._last_position = position
            self._position_cnt += 1

        return EventStatus.Succeeded()


def add_static_model_to_environment(
    model_item: Element, name: str, uri: str, X_WB: RigidTransform
) -> None:
    """Add a static model to an environment.

    This function creates and appends a static model to the provided
    environment, defined within an existing XML structure. It sets the model's
    name, URI, and pose based on the provided arguments.

    Args:
        model_item: The parent XML element to which the static model will
            be added.
        name: The name of the model.
        uri: The URI for the model, used to locate the model resource. We will
            always prepend 'package://underactuated/models/' to the provided URI.
        X_WB: A transformation representing the pose of the model.
    """
    include_item = SubElement(model_item, "include")
    # Add the name.
    name_item = SubElement(include_item, "name")
    name_item.text = name

    # Add the URI.
    uri_item = SubElement(include_item, "uri")
    uri_item.text = "package://underactuated/models/" + uri

    # Add the pose.
    static_item = SubElement(include_item, "static")
    static_item.text = "True"
    pose_item = SubElement(include_item, "pose")
    xyz = X_WB.translation()
    rpy = RollPitchYaw(X_WB.rotation()).vector()
    pose_item.text = f"{xyz[0]} {xyz[1]} {xyz[2]} {rpy[0]} {rpy[1]} {rpy[2]}"


class Direction(Enum):
    """The direction of the wall with respect to the cell center."""

    TOP = 0
    BOTTOM = 1
    LEFT = 2
    RIGHT = 3
    CENTER = 4


class Asset:
    """An asset that can be added to an environment."""

    def __init__(
        self,
        sdf_filename: str,
        X_WB: RigidTransform,
        collision_free_sets: Sequence[HPolyhedron],
    ) -> None:
        """Create an asset in the environment.

        Args:
            sdf_filename: The filename of the SDF file that defines the asset.
            X_WB: The pose of the asset in the world frame.
            collision_free_sets: A list of collision free sets associated with
                the asset.
        """
        self.sdf_filename = sdf_filename
        self.collision_free_sets = collision_free_sets
        self.X_WB = X_WB

    def add_to_environment(self, model_item: Element, name: str) -> None:
        """Add the asset to an environment.

        Args:
            model_item: The parent XML element to which the asset will
                be added.
            name: The name of the asset.
        """
        add_static_model_to_environment(model_item, name, self.sdf_filename, self.X_WB)


class Building(Asset):
    """A building asset in the environment."""

    # The minimum and maximum z heights for the building, which leave some
    # margin for the quad copter.
    MINIMUM_Z_HEIGHT = QUAD_COPTER_DIAMETER / 2
    MAXIMUM_Z_HEIGHT = BUILDING_HEIGHT - QUAD_COPTER_DIAMETER / 2
    SAFE_INDOOR_WIDTH = CELL_SIZE - WALL_THICKNESS - QUAD_COPTER_DIAMETER

    def __init__(
        self,
        sdf_filename: str,
        X_WB: RigidTransform,
        collision_free_sets: Sequence[HPolyhedron],
        direction: Direction,
    ) -> None:
        """Create a building asset in the environment.

        Args:
            sdf_filename: The filename of the SDF file that defines the asset.
            X_WB: The pose of the asset in the world frame.
            collision_free_sets: A list of collision free sets associated with
                the asset.
            direction: The direction in which the asset is facing to with
                recept of the cell center. See the diagram below.

                           TOP
                    ┌───────────────┐
                    │               │
                    │               │
             LEFT   │      Cell     │  RIGHT
                    │    Top View   │
                    │               │
                    │               │
                    └───────────────┘
                           BOTTOM

        """
        super().__init__(sdf_filename, X_WB, collision_free_sets)
        self.direction = direction

    @classmethod
    def make_internal_door(cls, x: float, y: float, direction: Direction) -> "Building":
        """Make a traversable internal door in a wall.

        │◄────── cell width ────────►│
        │                            │
        ├────────────────────────────┤
        │############################│
        │######┌──────────────┐######├──
        │######│              │######│ ▲
        │######│    ┌────┐    │######│ │
        │######│    │    │    │######│
        │######│    │    │    │######│door
        │######│    │    │    │######│height
        │######│    │    │    │######│
        │######│    └────┘    │######│ │
        │######│              │######│ ▼
        └──────┼────┬─────────┼──────┴──
               │    │         │
               │◄──►│         │
               │margin        │
               │              │
               │◄─door width─►│

        Args:
            x: The x coordinate of the wall.
            y: The y coordinate of the wall.
            direction: The direction of the wall.
        """
        sdf_filename = "uav_environment/wall_with_center_door_internal.sdf"

        door_width = 1.25  # meters.
        door_height = 2  # meters.

        # Make the collision free sets.
        regions = []
        match direction:
            case Direction.TOP | Direction.BOTTOM:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - (door_width - QUAD_COPTER_DIAMETER) / 2,
                            y - (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            cls.MINIMUM_Z_HEIGHT,
                        ],
                        [
                            x + (door_width - QUAD_COPTER_DIAMETER) / 2,
                            y + (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            door_height - QUAD_COPTER_DIAMETER / 2,
                        ],
                    )
                )

            case Direction.LEFT | Direction.RIGHT:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            y - (door_width - QUAD_COPTER_DIAMETER) / 2,
                            cls.MINIMUM_Z_HEIGHT,
                        ],
                        [
                            x + (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            y + (door_width - QUAD_COPTER_DIAMETER) / 2,
                            door_height - QUAD_COPTER_DIAMETER / 2,
                        ],
                    )
                )

        match direction:
            case Direction.TOP:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(-np.pi / 2)
                )
            case Direction.BOTTOM:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi / 2)
                )
            case Direction.LEFT:
                X_WB = RigidTransform(p=[x, y, 0], R=RotationMatrix.Identity())
            case Direction.RIGHT:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi)
                )

        return cls(sdf_filename, X_WB, regions, direction)

    @classmethod
    def make_internal_vertical_wall(
        cls, x: float, y: float, direction: Direction
    ) -> "Building":
        """Make a vertical wall.

        │◄────── cell width ────────►│
        │                            │
        ├────────────────────────────┼─▲─────
        │                            │ │    ▲
        │  ┌──────────────────────┐  ├─▼─   │
        │  │                      │  │margin
        │  └──────────────────────┘  │
        │                            │  building
        │############################│  height
        │############################│
        │############################│
        │############################│      │
        │############################│      ▼
        └────────────────────────────┴───────

        Args:
            x: The x coordinate of the center of the wall.
            y: The y coordinate of the center of the wall.
            direction: The direction of the wall.
        """
        sdf_filename = "uav_environment/half_wall_vertical.sdf"

        margin = WALL_THICKNESS + QUAD_COPTER_DIAMETER

        # Make the collision free sets.
        regions = []
        match direction:
            case Direction.TOP | Direction.BOTTOM:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - (CELL_SIZE - margin) / 2,
                            y - margin / 2,
                            (BUILDING_HEIGHT + QUAD_COPTER_DIAMETER) / 2,
                        ],
                        [
                            x + (CELL_SIZE - margin) / 2,
                            y + margin / 2,
                            cls.MAXIMUM_Z_HEIGHT,
                        ],
                    )
                )

            case Direction.LEFT | Direction.RIGHT:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - margin / 2,
                            y - (CELL_SIZE - margin) / 2,
                            (BUILDING_HEIGHT + QUAD_COPTER_DIAMETER) / 2,
                        ],
                        [
                            x + margin / 2,
                            y + (CELL_SIZE - margin) / 2,
                            cls.MAXIMUM_Z_HEIGHT,
                        ],
                    )
                )

        match direction:
            case Direction.TOP | Direction.BOTTOM:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi / 2)
                )
            case Direction.LEFT | Direction.RIGHT:
                X_WB = RigidTransform(p=[x, y, 0], R=RotationMatrix.Identity())

        return cls(sdf_filename, X_WB, regions, direction)

    @classmethod
    def make_internal_horizontal_wall_right(
        cls, x: float, y: float, direction: Direction
    ) -> "Building":
        """Make an internal horizontal wall with a blockage on the right.

        ├─────── cell width ────────►│
        │                            │
        ├────────────────────────────┼─▲─────
        │              ##############│ │    ▲
        │  ┌───────┐   ##############├─▼─   │
        │  │       │   ##############│margin
        │  │       │   ##############│
        │  │       │   ##############│  building
        │  │       │   ##############│  height
        │  │       │   ##############│
        │  │       │   ##############│
        │  └───────┘   ##############│      │
        │              ##############│      ▼
        └────────────────────────────┴───────


        Args:
            x: The x coordinate of the center of the wall.
            y: The y coordinate of the center of the wall.
            direction: The direction of the wall.
        """
        sdf_filename = "uav_environment/half_wall_horizontal_mirror.sdf"

        margin = WALL_THICKNESS + QUAD_COPTER_DIAMETER
        # Make the collision free sets.
        regions = []
        match direction:
            case Direction.TOP | Direction.BOTTOM:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - (CELL_SIZE - margin) / 2,
                            y - margin / 2,
                            cls.MINIMUM_Z_HEIGHT,
                        ],
                        [
                            x + -margin / 2,
                            y + margin / 2,
                            cls.MAXIMUM_Z_HEIGHT,
                        ],
                    )
                )

            case Direction.LEFT | Direction.RIGHT:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - margin / 2,
                            y - (CELL_SIZE - margin) / 2,
                            cls.MINIMUM_Z_HEIGHT,
                        ],
                        [
                            x + margin / 2,
                            y + -margin / 2,
                            cls.MAXIMUM_Z_HEIGHT,
                        ],
                    )
                )

        match direction:
            case Direction.TOP | Direction.BOTTOM:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi / 2)
                )
            case Direction.LEFT | Direction.RIGHT:
                X_WB = RigidTransform(p=[x, y, 0], R=RotationMatrix.Identity())

        return cls(sdf_filename, X_WB, regions, direction)

    @classmethod
    def make_internal_horizontal_wall_left(
        cls, x: float, y: float, direction: Direction
    ) -> "Building":
        """Make an internal horizontal wall with a blockage on the left.

        ├─────── cell width ────────►│
        │                            │
        ├────────────────────────────┼─▲─────
        │##############              │ │    ▲
        │##############   ┌───────┐  ├─▼─   │
        │##############   │       │  │margin
        │##############   │       │  │
        │##############   │       │  │  building
        │##############   │       │  │  height
        │##############   │       │  │
        │##############   │       │  │
        │##############   └───────┘  │      │
        │##############              │      ▼
        └────────────────────────────┴───────


        Args:
            x: The x coordinate of the center of the wall.
            y: The y coordinate of the center of the wall.
            direction: The direction of the wall.
        """
        sdf_filename = "uav_environment/half_wall_horizontal.sdf"

        margin = WALL_THICKNESS + QUAD_COPTER_DIAMETER
        # Make the collision free sets.
        regions = []
        match direction:
            case Direction.TOP | Direction.BOTTOM:
                regions.append(
                    HPolyhedron.MakeBox(
                        [x + margin / 2, y - margin / 2, cls.MINIMUM_Z_HEIGHT],
                        [
                            x + (CELL_SIZE - margin) / 2,
                            y + margin / 2,
                            cls.MAXIMUM_Z_HEIGHT,
                        ],
                    )
                )

            case Direction.LEFT | Direction.RIGHT:
                regions.append(
                    HPolyhedron.MakeBox(
                        [x - margin / 2, y + margin / 2, cls.MINIMUM_Z_HEIGHT],
                        [
                            x + margin / 2,
                            y + (CELL_SIZE - margin) / 2,
                            cls.MAXIMUM_Z_HEIGHT,
                        ],
                    )
                )

        match direction:
            case Direction.TOP:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi / 2)
                )
            case Direction.BOTTOM:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi / 2)
                )
            case Direction.LEFT:
                X_WB = RigidTransform(p=[x, y, 0], R=RotationMatrix.Identity())
            case Direction.RIGHT:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi)
                )

        return cls(sdf_filename, X_WB, regions, direction)

    @classmethod
    def make_internal_no_wall(
        cls, x: float, y: float, direction: Direction
    ) -> "Building":
        """No internal_wall at all.

        While this is adding no wall, it is still adding a collision free set.

        │◄────── cell width ────────►│
        │                            │
        ├────────────────────────────┤
        │                            │
        │   ┌────────────────────┐   │
        │   │                    │   │
        │   │                    │   │
        │   │                    │   │
        │   │                    │   │
        │   │                    │   │
        │   │                    │   │
        │   └────────────────────┘   │
        │                            │
        ├────┬───────────────────────┘
        │    │
        │◄──►│
         margin


        Args:
            x: The x coordinate of the center of no wall.
            y: The y coordinate of the center of no wall.
            direction: The direction of the no wall.
        """
        margin = WALL_THICKNESS + QUAD_COPTER_DIAMETER
        # Make the collision free sets.
        regions = []
        match direction:
            case Direction.TOP | Direction.BOTTOM:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - (CELL_SIZE - margin) / 2,
                            y - margin / 2,
                            cls.MINIMUM_Z_HEIGHT,
                        ],
                        [
                            x + (CELL_SIZE - margin) / 2,
                            y + margin / 2,
                            cls.MAXIMUM_Z_HEIGHT,
                        ],
                    )
                )

            case Direction.LEFT | Direction.RIGHT:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - margin / 2,
                            y - (CELL_SIZE - margin) / 2,
                            cls.MINIMUM_Z_HEIGHT,
                        ],
                        [
                            x + margin / 2,
                            y + (CELL_SIZE - margin) / 2,
                            cls.MAXIMUM_Z_HEIGHT,
                        ],
                    )
                )

        match direction:
            case Direction.TOP:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(-np.pi / 2)
                )
            case Direction.BOTTOM:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi / 2)
                )
            case Direction.LEFT:
                X_WB = RigidTransform(p=[x, y, 0], R=RotationMatrix.Identity())
            case Direction.RIGHT:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi)
                )

        return cls("", X_WB, regions, direction)

    @classmethod
    def make_external_door(cls, x: float, y: float, direction: Direction) -> "Building":
        """Make a traversable door in a wall.

        │◄────── cell width ────────►│
        │                            │
        ├────────────────────────────┤
        │############################│
        │######┌──────────────┐######├──
        │######│              │######│ ▲
        │######│    ┌────┐    │######│ │
        │######│    │    │    │######│
        │######│    │    │    │######│door
        │######│    │    │    │######│height
        │######│    │    │    │######│
        │######│    └────┘    │######│ │
        │######│              │######│ ▼
        └──────┼────┬─────────┼──────┴──
               │    │         │
               │◄──►│         │
               │margin        │
               │              │
               │◄─door width─►│

        Args:
            x: The x coordinate of the center of the door.
            y: The y coordinate of the center of the door.
            direction: The direction of the wall the door is in.
        """
        sdf_filename = "uav_environment/wall_with_center_door.sdf"

        door_width = 1.25  # meters.
        door_height = 2  # meters.

        # Make the collision free sets.
        regions = []
        match direction:
            case Direction.TOP | Direction.BOTTOM:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - (door_width - QUAD_COPTER_DIAMETER) / 2,
                            y - (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            cls.MINIMUM_Z_HEIGHT,
                        ],
                        [
                            x + (door_width - QUAD_COPTER_DIAMETER) / 2,
                            y + (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            door_height - QUAD_COPTER_DIAMETER / 2,
                        ],
                    )
                )

            case Direction.LEFT | Direction.RIGHT:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            y - (door_width - QUAD_COPTER_DIAMETER) / 2,
                            cls.MINIMUM_Z_HEIGHT,
                        ],
                        [
                            x + (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            y + (door_width - QUAD_COPTER_DIAMETER) / 2,
                            door_height - QUAD_COPTER_DIAMETER / 2,
                        ],
                    )
                )

        match direction:
            case Direction.TOP:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(-np.pi / 2)
                )
            case Direction.BOTTOM:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi / 2)
                )
            case Direction.LEFT:
                X_WB = RigidTransform(p=[x, y, 0], R=RotationMatrix.Identity())
            case Direction.RIGHT:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi)
                )

        return cls(sdf_filename, X_WB, regions, direction)

    @classmethod
    def make_external_window_left(
        cls, x: float, y: float, direction: Direction
    ) -> "Building":
        """Make external wall with a window on the left.

        ├─────── cell width ─────────┤
        │                            │
        ├────────────────────────────┼────────
        │############################│       ▲
        │##┌─────────┐###############│       │
        │##│         │###############│
        │##│ ┌─────┐ │###############│
        │##│ │     │ │###############│  building
        │##│ │     │ │###############│  height
        │##│ └─────┘ │###############├─▲─
        │##│         │###############│ │
        │##└─────────┘###############├─▼─    │
        │############################│margin ▼
        └──┬─────────┬───────────────┴────────
           │  window │
           │◄─width─►│

        Args:
            x: The x coordinate of the center of the wall.
            y: The y coordinate of the center of the wall.
            direction: The direction of the wall.
        """
        sdf_filename = "uav_environment/wall_with_left_window.sdf"

        window_width = 1.5  # meters.
        window_offset = CELL_SIZE / 4
        margin = window_width - QUAD_COPTER_DIAMETER

        minimum_z_height_window = (BUILDING_HEIGHT - margin) / 2
        maximum_z_height_window = BUILDING_HEIGHT - (BUILDING_HEIGHT - margin) / 2

        regions = []
        match direction:
            case Direction.TOP:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x + window_offset - margin / 2,
                            y - (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            minimum_z_height_window,
                        ],
                        [
                            x + window_offset + margin / 2,
                            y + (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            maximum_z_height_window,
                        ],
                    )
                )
            case Direction.BOTTOM:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - window_offset - margin / 2,
                            y - (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            minimum_z_height_window,
                        ],
                        [
                            x - window_offset + margin / 2,
                            y + (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            maximum_z_height_window,
                        ],
                    )
                )

            case Direction.LEFT:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            y + window_offset - margin / 2,
                            minimum_z_height_window,
                        ],
                        [
                            x + (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            y + window_offset + margin / 2,
                            maximum_z_height_window,
                        ],
                    )
                )
            case Direction.RIGHT:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            y - window_offset - margin / 2,
                            minimum_z_height_window,
                        ],
                        [
                            x + (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            y - window_offset + margin / 2,
                            maximum_z_height_window,
                        ],
                    )
                )

        match direction:
            case Direction.TOP:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(-np.pi / 2)
                )
            case Direction.BOTTOM:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi / 2)
                )
            case Direction.LEFT:
                X_WB = RigidTransform(p=[x, y, 0], R=RotationMatrix.Identity())
            case Direction.RIGHT:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi)
                )

        return cls(sdf_filename, X_WB, regions, direction)

    @classmethod
    def make_external_window_right(
        cls, x: float, y: float, direction: Direction
    ) -> "Building":
        """Make an external wall with a window on the right.

        ├─────── cell width ─────────┤
        │                            │
        ├────────────────────────────┼────────
        │############################│       ▲
        │###############┌─────────┐##│       │
        │###############│         │##│
        │###############│ ┌─────┐ │##│
        │###############│ │     │ │##│  building
        │###############│ │     │ │##│  height
        │###############│ └─────┘ │##├─▲─
        │###############│         │##│ │
        │###############└─────────┘##├─▼─    │
        │############################│margin │
        └───────────────┬─────────┬──┴───────┴
                        │  window │
                        │◄─width─►│

        Args:
            x: The x coordinate of the center of the wall.
            y: The y coordinate of the center of the wall.
            direction: The direction of the wall.
        """
        sdf_filename = "uav_environment/wall_with_right_window.sdf"

        window_width = 1.5  # meters.
        window_offset = CELL_SIZE / 4
        margin = window_width - QUAD_COPTER_DIAMETER

        minimum_z_height_window = (BUILDING_HEIGHT - margin) / 2
        maximum_z_height_window = BUILDING_HEIGHT - (BUILDING_HEIGHT - margin) / 2

        regions = []
        match direction:
            case Direction.TOP:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - window_offset - margin / 2,
                            y - (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            minimum_z_height_window,
                        ],
                        [
                            x - window_offset + margin / 2,
                            y + (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            maximum_z_height_window,
                        ],
                    )
                )
            case Direction.BOTTOM:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x + window_offset - margin / 2,
                            y - (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            minimum_z_height_window,
                        ],
                        [
                            x + window_offset + margin / 2,
                            y + (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            maximum_z_height_window,
                        ],
                    )
                )

            case Direction.LEFT:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            y - window_offset - margin / 2,
                            minimum_z_height_window,
                        ],
                        [
                            x + (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            y - window_offset + margin / 2,
                            maximum_z_height_window,
                        ],
                    )
                )
            case Direction.RIGHT:
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            y + window_offset - margin / 2,
                            minimum_z_height_window,
                        ],
                        [
                            x + (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            y + window_offset + margin / 2,
                            maximum_z_height_window,
                        ],
                    )
                )
        match direction:
            case Direction.TOP:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(-np.pi / 2)
                )
            case Direction.BOTTOM:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi / 2)
                )
            case Direction.LEFT:
                X_WB = RigidTransform(p=[x, y, 0], R=RotationMatrix.Identity())
            case Direction.RIGHT:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi)
                )

        return cls(sdf_filename, X_WB, regions, direction)

    @classmethod
    def make_external_windows(
        cls, x: float, y: float, direction: Direction
    ) -> "Building":
        """Make an external wall with two windows.

        ├─────── cell width ─────────┤
        │                            │
        ├────────────────────────────┼────────
        │############################│       ▲
        │##┌─────────┐##┌─────────┐##│       │
        │##│         │##│         │##│
        │##│ ┌─────┐ │##│ ┌─────┐ │##│
        │##│ │     │ │##│ │     │ │##│  building
        │##│ │     │ │##│ │     │ │##│  height
        │##│ └─────┘ │##│ └─────┘ │##├─▲─
        │##│         │##│         │##│ │
        │##└─────────┘##└─────────┘##├─▼─    │
        │############################│margin │
        └──┬─────────┬──┬─────────┬──┴───────┴
           │  window │  │  window │
           │◄─width─►│  │◄─width─►│


        Args:
            x: The x coordinate of the center of the wall.
            y: The y coordinate of the center of the wall.
            direction: The direction of the wall.
        """
        sdf_filename = "uav_environment/wall_with_windows.sdf"

        window_width = 1.5  # meters.
        window_offset = CELL_SIZE / 4
        margin = window_width - QUAD_COPTER_DIAMETER

        minimum_z_height_window = (BUILDING_HEIGHT - margin) / 2
        maximum_z_height_window = BUILDING_HEIGHT - (BUILDING_HEIGHT - margin) / 2

        regions = []

        match direction:
            case Direction.TOP | Direction.BOTTOM:
                # Right window.
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x + window_offset - margin / 2,
                            y - (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            minimum_z_height_window,
                        ],
                        [
                            x + window_offset + margin / 2,
                            y + (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            maximum_z_height_window,
                        ],
                    )
                )

                # Left window.
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - window_offset - margin / 2,
                            y - (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            minimum_z_height_window,
                        ],
                        [
                            x - window_offset + margin / 2,
                            y + (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            maximum_z_height_window,
                        ],
                    )
                )

            case Direction.LEFT | Direction.RIGHT:
                # Right window.
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            y + window_offset - margin / 2,
                            minimum_z_height_window,
                        ],
                        [
                            x + (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            y + window_offset + margin / 2,
                            maximum_z_height_window,
                        ],
                    )
                )

                # Left window.
                regions.append(
                    HPolyhedron.MakeBox(
                        [
                            x - (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            y - window_offset - margin / 2,
                            minimum_z_height_window,
                        ],
                        [
                            x + (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2,
                            y - window_offset + margin / 2,
                            maximum_z_height_window,
                        ],
                    )
                )

        match direction:
            case Direction.TOP:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(-np.pi / 2)
                )
            case Direction.BOTTOM:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi / 2)
                )
            case Direction.LEFT:
                X_WB = RigidTransform(p=[x, y, 0], R=RotationMatrix.Identity())
            case Direction.RIGHT:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi)
                )

        return cls(sdf_filename, X_WB, regions, direction)

    @classmethod
    def make_external_wall(cls, x: float, y: float, direction: Direction) -> "Building":
        """Non-traversable wall for the building facade.

        The collision free set will be empty since the wall can't be traversed.

        │◄────── cell width ────────►│
        │                            │
        ├────────────────────────────┤
        │############################│
        │############################│
        │############################│
        │############################│
        │############################│
        │############################│
        │############################│
        │############################│
        │############################│
        │############################│
        └────────────────────────────┘

        Args:
            x: The x coordinate of the center of no wall.
            y: The y coordinate of the center of no wall.
            direction: The direction of the no wall.
        """
        sdf_filename = "uav_environment/just_wall.sdf"

        match direction:
            case Direction.TOP:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(-np.pi / 2)
                )
            case Direction.BOTTOM:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi / 2)
                )
            case Direction.LEFT:
                X_WB = RigidTransform(p=[x, y, 0], R=RotationMatrix.Identity())
            case Direction.RIGHT:
                X_WB = RigidTransform(
                    p=[x, y, 0], R=RotationMatrix.MakeZRotation(np.pi)
                )

        return cls(sdf_filename, X_WB, [], direction)


class OutdoorDecoration(Asset):
    """An outdoor decoration that can be added to an environment.."""

    @staticmethod
    def compute_cell_bounds(
        x: float, y: float, indoor_neighbor_directions: Sequence[Direction]
    ) -> Tuple[np.array, np.array]:
        """Compute the bounds of the collision free set for a cell.

        Starting with the cell bounds to be the entire cell, we shrink the
        space for each direction where the cell is considered indoor by the wall
        thickness and quad copter margin.

        Args:
            x: The x coordinate of the center of the cell.
            y: The y coordinate of the center of the cell.
            indoor_neighbor_directions: The directions of the neighbors that are
                indoor.

        Returns:
            A tuple of the lower and upper bounds of the collision free region.
        """
        # Create lower and upper bound of collision free set.
        lb = np.array([x - CELL_SIZE / 2, y - CELL_SIZE / 2, QUAD_COPTER_DIAMETER / 2])
        ub = np.array(
            [
                x + CELL_SIZE / 2,
                y + CELL_SIZE / 2,
                BUILDING_HEIGHT - QUAD_COPTER_DIAMETER / 2,
            ]
        )

        # Shrink the bounds for each indoor neighbor.
        if Direction.RIGHT in indoor_neighbor_directions:
            ub[0] -= (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2
        if Direction.LEFT in indoor_neighbor_directions:
            lb[0] += (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2
        if Direction.TOP in indoor_neighbor_directions:
            ub[1] -= (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2
        if Direction.BOTTOM in indoor_neighbor_directions:
            lb[1] += (WALL_THICKNESS + QUAD_COPTER_DIAMETER) / 2
        return lb, ub

    @classmethod
    def make_nothing(
        cls,
        x: float,
        y: float,
        indoor_neighbor_directions: Sequence[Direction],
    ) -> "OutdoorDecoration":
        """Create an empty outdoor space with no assets.

        Args:
            x: The x coordinate of the center of the outdoor space.
            y: The y coordinate of the center of no wall.
            indoor_neighbor_directions: A cell can have neighbors which may be
                of type indoor. This is a list of directions of all the
                neighbors that are of type indoor.
        """
        sdf_filename = ""

        lb, ub = cls.compute_cell_bounds(x, y, indoor_neighbor_directions)
        regions = [HPolyhedron.MakeBox(lb, ub)]

        return cls(sdf_filename, RigidTransform(), regions)

    @classmethod
    def make_tree(
        cls,
        x: float,
        y: float,
        indoor_neighbor_directions: Sequence[Direction],
    ) -> "OutdoorDecoration":
        """Create an outdoor space wth a tree at a random location.

        The surrounding boxes represent the collision free sets.
        ┌──────────────────────────────┐
        │                              │
        │                              │
        │                              │
        │                              │
        ├───┬────────────┬─────────────┤
        │   │            │             │
        │   │  ┌──────┐  │             │
        │   │  │      │  │             │
        │   │  │ Tree │  │             │
        │   │  │      │  │             │
        │   │  └──────┘  │             │
        │   │            │             │
        ├───┴────────────┴─────────────┤
        │                              │
        │                              │
        └───┬──┬───────────────────────┘
            │  │
            ◄──► margin


        Args:
            x: The x coordinate of the center of the outdoor space.
            y: The y coordinate of the center of no wall.
            indoor_neighbor_directions: A cell can have neighbors which may be
                of type indoor. This is a list of directions of all the
                neighbors that are of type indoor.
        """
        sdf_filename = "uav_environment/tree.sdf"
        tree_width = 1.0

        lb, ub = cls.compute_cell_bounds(x, y, indoor_neighbor_directions)

        # Generate a random position for the tree.
        tree_x = x + np.random.uniform(
            -(CELL_SIZE - tree_width) / 2, (CELL_SIZE - tree_width) / 2
        )
        tree_y = y + np.random.uniform(
            -(CELL_SIZE - tree_width) / 2, (CELL_SIZE - tree_width) / 2
        )

        regions = []

        if tree_y - (tree_width + QUAD_COPTER_DIAMETER) / 2 > lb[1]:
            # There is enough space to the bottom of the tree.
            regions.append(
                HPolyhedron.MakeBox(
                    lb,
                    [
                        ub[0],
                        tree_y - (tree_width + QUAD_COPTER_DIAMETER) / 2,
                        ub[2],
                    ],
                )
            )
        if tree_y + (tree_width + QUAD_COPTER_DIAMETER) / 2 < ub[1]:
            # There is enough space to the top of the tree.
            regions.append(
                HPolyhedron.MakeBox(
                    [
                        lb[0],
                        tree_y + (tree_width + QUAD_COPTER_DIAMETER) / 2,
                        lb[2],
                    ],
                    ub,
                )
            )
        if tree_x - (tree_width + QUAD_COPTER_DIAMETER) / 2 > lb[0]:
            # There is enough space to the left of the tree.
            regions.append(
                HPolyhedron.MakeBox(
                    lb,
                    [
                        tree_x - (tree_width + QUAD_COPTER_DIAMETER) / 2,
                        ub[1],
                        ub[2],
                    ],
                )
            )
        if tree_x + (tree_width + QUAD_COPTER_DIAMETER) / 2 < ub[0]:
            # There is enough space to the right of the tree.
            regions.append(
                HPolyhedron.MakeBox(
                    [
                        tree_x + (tree_width + QUAD_COPTER_DIAMETER) / 2,
                        lb[1],
                        lb[2],
                    ],
                    ub,
                )
            )

        X_WB = RigidTransform(p=[tree_x, tree_y, 0])

        return cls(sdf_filename, X_WB, regions)


class CellType(Enum):
    """The type of the cell in the grid world."""

    UNDECIDED = 0
    INDOOR = 1
    OUTDOOR = 2


class UavEnvironment:
    """A three dimensional environment for a UAV to fly in."""

    # The default start and goal points.
    DEFAULT_START = (0.0, 0.0, 2.0)
    DEFAULT_GOAL = (15.0, 15.0, 0.5)

    # The probability of each cell type, which are in the order of:
    # [undecided, indoor, outdoor]
    CELL_TYPE_SAMPLING_PROBABILITIES = [0.0, 0.7, 0.3]

    # The color of the collision free sets used in meshcat.
    IRIS_REGION_COLOR = Rgba(0.698, 0.67, 1, 0.4)

    # The color for the edges between regions used in meshcat.
    EDGES_BETWEEN_REGIONS_COLOR = Rgba(0.8, 0.8, 0.5, 0.4)

    # The amount of time to wait between trajectories.
    DELAY_BETWEEN_TRAJECTORIES = 1.0  # seconds.

    # The number of samples to use for persistent traces.
    NUM_SAMPLES = 1000

    # The width of the lines used to draw lines in meshcat.
    LINE_WIDTH = 5.0

    def __init__(
        self, environment_shape: Tuple[int, int] = (5, 5), seed: int = 0
    ) -> None:
        """Initialize the environment separated in a grid.

        Each cell in the grid is 5x5 meters and can be either indoor or outdoor.

        At the moment the start and goal are set. The start cell
        will be set to type outdoor and the goal cell will be set to type
        indoor.

        Args:
            environment_shape: The shape of the environment grid world.
            seed: The random seed to use when generating the environment.
        """
        # Set random seed.
        np.random.seed(seed)
        # Fill the grid with undecided cells.
        self.shape = environment_shape
        self.grid = np.full(self.shape, CellType.UNDECIDED)

        # The edge of the world is always outdoor.
        self.grid[:, 0] = CellType.OUTDOOR
        self.grid[:, -1] = CellType.OUTDOOR
        self.grid[0, :] = CellType.OUTDOOR
        self.grid[-1, :] = CellType.OUTDOOR

        # Make the default start outdoor and the default goal indoor.
        # TODO(wrangelvid): Allow the user to specify the start and goal.
        self.grid[0, 0] = CellType.OUTDOOR
        self.grid[2, 1] = CellType.INDOOR

        queue = [(1, 1)]
        while queue:
            position = queue.pop(0)

            for dx, dy in zip([-1, 1, 0, 0], [0, 0, -1, 1]):
                neighbor = (position[0] + dx, position[1] + dy)

                # If the cell neighbor is out of bounds, skip it.
                if (
                    neighbor[0] < 0
                    or neighbor[1] < 0
                    or neighbor[0] >= self.grid.shape[0]
                    or neighbor[1] >= self.grid.shape[1]
                ):
                    continue

                # If the cell neighbor is already decided, skip it.
                if self.grid[neighbor] != CellType.UNDECIDED:
                    continue

                # Choose a cell type for the neighbor.
                self.grid[neighbor] = np.random.choice(
                    CellType, p=self.CELL_TYPE_SAMPLING_PROBABILITIES
                )
                # Expand the queue.
                queue.append(neighbor)

        self.tree: Optional[ElementTree] = None
        self.sdf_as_string = ""
        self.edges_between_regions: Set[Tuple[int, int]] = set()
        self.all_regions: List[HPolyhedron] = []

        self.default_indoor_building_assets = {
            Building.make_internal_horizontal_wall_right: 0.25,
            Building.make_internal_horizontal_wall_left: 0.25,
            Building.make_internal_vertical_wall: 0.125,
            Building.make_internal_door: 0.25,
            Building.make_internal_no_wall: 0.125,
        }

        self.default_outdoor_building_assets = {
            Building.make_external_windows: 0.01,
            Building.make_external_window_right: 0.05,
            Building.make_external_window_left: 0.05,
            Building.make_external_door: 0.09,
            Building.make_external_wall: 0.8,
        }

        self.default_outdoor_assets = {
            OutdoorDecoration.make_tree: 0.3,
            OutdoorDecoration.make_nothing: 0.7,
        }

    def compile(self) -> Tuple[List[HPolyhedron], List[Tuple[int, int]]]:
        """Compile the grid into an environment with buildings.

        Returns
            A list of collision free sets that represents the environment.
            A list of edges between these regions that are represented as tuples
            with indices into the list of collision free sets.
        """
        root_item = Element("sdf", version="1.5", nsmap={"drake": "drake.mit.edu"})
        model_item = SubElement(root_item, "model", name="building")

        regions: Dict[int, Dict[int, List[HPolyhedron]]] = dict()
        for x_idx in range(self.grid.shape[0]):
            regions[x_idx] = dict()
            for y_idx in range(self.grid.shape[1]):
                regions[x_idx][y_idx] = []

                # Center of the cell.
                p_Wo_cell = np.array([x_idx, y_idx, 0]) * CELL_SIZE
                X_Cell = RigidTransform(p=p_Wo_cell)

                # Find valid neighbors.
                neighbor_positions = {}
                if x_idx > 0:
                    neighbor_positions[Direction.LEFT] = (x_idx - 1, y_idx)
                if y_idx > 0:
                    neighbor_positions[Direction.BOTTOM] = (x_idx, y_idx - 1)
                if x_idx < self.grid.shape[0] - 1:
                    neighbor_positions[Direction.RIGHT] = (x_idx + 1, y_idx)
                if y_idx < self.grid.shape[1] - 1:
                    neighbor_positions[Direction.TOP] = (x_idx, y_idx + 1)

                match self.grid[x_idx, y_idx]:
                    case CellType.INDOOR:
                        add_static_model_to_environment(
                            model_item,
                            f"indoor/floor/{x_idx}_{y_idx}",
                            "uav_environment/floor_indoor.sdf",
                            X_Cell,
                        )
                        add_static_model_to_environment(
                            model_item,
                            f"indoor/ceiling/{x_idx}_{y_idx}",
                            "uav_environment/ceiling.sdf",
                            X_Cell,
                        )

                        # Add collision free sets.
                        regions[x_idx][y_idx].append(
                            HPolyhedron.MakeBox(
                                [
                                    p_Wo_cell[0] - Building.SAFE_INDOOR_WIDTH / 2,
                                    p_Wo_cell[1] - Building.SAFE_INDOOR_WIDTH / 2,
                                    Building.MINIMUM_Z_HEIGHT,
                                ],
                                [
                                    p_Wo_cell[0] + Building.SAFE_INDOOR_WIDTH / 2,
                                    p_Wo_cell[1] + Building.SAFE_INDOOR_WIDTH / 2,
                                    Building.MAXIMUM_Z_HEIGHT,
                                ],
                            )
                        )

                        for direction, (
                            neighbor_x_idx,
                            neighbor_y_idx,
                        ) in neighbor_positions.items():
                            p_Wo_neighbor = (
                                np.array([neighbor_x_idx, neighbor_y_idx, 0])
                                * CELL_SIZE
                            )
                            p_Wo_wall = p_Wo_cell + (p_Wo_neighbor - p_Wo_cell) / 2

                            if (
                                self.grid[neighbor_x_idx, neighbor_y_idx]
                                is CellType.OUTDOOR
                            ):
                                # The neighbor is in a different cell type,
                                # so we need to add a outdoor wall between them.
                                outdoor_building_asset_creation_function = np.random.choice(
                                    list(self.default_outdoor_building_assets.keys()),
                                    p=list(
                                        self.default_outdoor_building_assets.values()
                                    ),
                                )
                                # Create asset.
                                asset = outdoor_building_asset_creation_function(
                                    p_Wo_wall[0], p_Wo_wall[1], direction
                                )
                                asset.add_to_environment(
                                    model_item,
                                    f"outdoor/wall/{x_idx}_{y_idx}_{direction.value}",
                                )
                                regions[x_idx][y_idx] += asset.collision_free_sets

                            if self.grid[
                                neighbor_x_idx, neighbor_y_idx
                            ] is CellType.INDOOR and direction in [
                                Direction.RIGHT,
                                Direction.TOP,
                            ]:
                                indoor_asset_creation_function = np.random.choice(
                                    list(self.default_indoor_building_assets.keys()),
                                    p=list(
                                        self.default_indoor_building_assets.values()
                                    ),
                                )

                                # Create asset.
                                asset = indoor_asset_creation_function(
                                    p_Wo_wall[0], p_Wo_wall[1], direction
                                )
                                regions[x_idx][y_idx] += asset.collision_free_sets
                                if asset.sdf_filename:
                                    asset.add_to_environment(
                                        model_item,
                                        f"indoor/wall/{x_idx}_{y_idx}_{direction.value}",
                                    )

                    case CellType.OUTDOOR:
                        add_static_model_to_environment(
                            model_item,
                            f"outdoor/floor/{x_idx}_{y_idx}",
                            "uav_environment/floor_outdoor.sdf",
                            X_Cell,
                        )

                        outdoor_asset_creation_function = np.random.choice(
                            list(self.default_outdoor_assets.keys()),
                            p=list(self.default_outdoor_assets.values()),
                        )

                        indoor_neighbor_directions = []
                        for direction, (
                            neighbor_x_idx,
                            neighbor_y_idx,
                        ) in neighbor_positions.items():
                            if (
                                self.grid[neighbor_x_idx, neighbor_y_idx]
                                == CellType.INDOOR
                            ):
                                indoor_neighbor_directions.append(direction)

                        # Create asset.
                        asset = outdoor_asset_creation_function(
                            p_Wo_cell[0],
                            p_Wo_cell[1],
                            indoor_neighbor_directions,
                        )
                        regions[x_idx][y_idx] += asset.collision_free_sets

                        if asset.sdf_filename:
                            asset.add_to_environment(
                                model_item,
                                f"outdoor/decoration/{x_idx}_{y_idx}",
                            )

        self.tree = ElementTree(root_item)
        self.sdf_as_string = tostring(self.tree, encoding="utf8").decode()

        # Compute edges between regions.
        self.edges_between_regions = set()
        self.all_regions = []
        map_region_coordinates = dict()

        # Flatten the regions into a list and create a map from region
        # coordinates to the index of the region in the flattened list.
        cnt = 0
        for x_idx in range(self.grid.shape[0]):
            for y_idx in range(self.grid.shape[1]):
                self.all_regions += regions[x_idx][y_idx]
                for i in range(len(regions[x_idx][y_idx])):
                    map_region_coordinates[(x_idx, y_idx, i)] = cnt
                    cnt += 1

        # Perform intersections checks.
        for x_idx in range(self.grid.shape[0]):
            for y_idx in range(self.grid.shape[1]):

                # compute edges within a cell.
                for i in range(len(regions[x_idx][y_idx]) - 1):
                    for j in range(i + 1, len(regions[x_idx][y_idx])):
                        if regions[x_idx][y_idx][i].IntersectsWith(
                            regions[x_idx][y_idx][j]
                        ):
                            self.edges_between_regions.add(
                                (
                                    map_region_coordinates[(x_idx, y_idx, i)],
                                    map_region_coordinates[(x_idx, y_idx, j)],
                                )
                            )
                            self.edges_between_regions.add(
                                (
                                    map_region_coordinates[(x_idx, y_idx, j)],
                                    map_region_coordinates[(x_idx, y_idx, i)],
                                )
                            )

                # Find intersections with neighbor cells.
                neighbor_positions = {}
                if x_idx < self.grid.shape[0] - 1:
                    neighbor_positions[Direction.RIGHT] = (x_idx + 1, y_idx)
                if y_idx < self.grid.shape[1] - 1:
                    neighbor_positions[Direction.BOTTOM] = (x_idx, y_idx + 1)

                for direction, (
                    neighbor_x_idx,
                    neighbor_y_idx,
                ) in neighbor_positions.items():
                    for i, region in enumerate(regions[x_idx][y_idx]):
                        for j, neighbor_region in enumerate(
                            regions[neighbor_x_idx][neighbor_y_idx]
                        ):
                            if region.IntersectsWith(neighbor_region):
                                self.edges_between_regions.add(
                                    (
                                        map_region_coordinates[(x_idx, y_idx, i)],
                                        map_region_coordinates[
                                            (neighbor_x_idx, neighbor_y_idx, j)
                                        ],
                                    )
                                )
                                self.edges_between_regions.add(
                                    (
                                        map_region_coordinates[
                                            (neighbor_x_idx, neighbor_y_idx, j)
                                        ],
                                        map_region_coordinates[(x_idx, y_idx, i)],
                                    )
                                )
        return self.all_regions, list(self.edges_between_regions)

    def animate_trajectory(
        self,
        meshcat: Meshcat,
        trajectories: Sequence[CompositeTrajectory] = [],
        fly_in_sequence: bool = True,
        quadrotor_separation: Optional[float] = None,
    ) -> None:
        """Visualize the environment with drones following the trajectories.

        The simulation is ignoring the physics and is setting the visual
        geometry of the quadrotor to what the flatness inverter would compute.

        Args:
            meshcat: The meshcat instance to use for visualization.
            trajectories: A list of trajectories to visualize.
            fly_in_sequence: If true, the drones will fly one by one, otherwise
                they will fly all at the same time.
            quadrotor_separation: The time between each quadrotor taking off,
                used to visalize the motion in a still picture.
        """
        meshcat.Delete()
        meshcat.DeleteAddedControls()
        meshcat.SetProperty("/Background", "visible", False)
        meshcat.SetProperty("/Grid", "visible", False)
        meshcat.SetProperty("/Axes", "visible", False)
        meshcat.SetProperty("/Lights/PointLightPositiveX", "visible", False)
        meshcat.SetProperty("/Lights/PointLightNegativeX", "visible", False)
        meshcat.SetProperty("/Lights/AmbientLight/<object>", "intensity", 1.3)

        # Parse the environment.
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
        parser = Parser(plant)
        ConfigureParser(parser)
        parser.AddModelsFromString(self.sdf_as_string, "sdf")
        plant.Finalize()

        ApplyVisualizationConfig(
            config=VisualizationConfig(publish_inertia=False, publish_contacts=False),
            plant=plant,
            scene_graph=scene_graph,
            builder=builder,
            meshcat=meshcat,
        )

        trajectory_colors = (
            cmap_ibm(np.linspace(0, 1, len(trajectories))) if trajectories else []
        )

        previous_trajectory_end_time = 0.0
        total_animation_duration = 0.0
        for i, traj in enumerate(trajectories):
            if fly_in_sequence:
                traj = shift_composite_trajectory(traj, previous_trajectory_end_time)
                previous_trajectory_end_time = (
                    traj.end_time() + self.DELAY_BETWEEN_TRAJECTORIES
                )
            total_animation_duration = max(total_animation_duration, traj.end_time())

            # Visualize the quadrotor.
            if quadrotor_separation:
                offsets = np.linspace(
                    traj.start_time(),
                    traj.end_time(),
                    int((traj.end_time() - traj.start_time()) / quadrotor_separation),
                )
                for j, offset in enumerate(offsets):
                    flat_traj_source = builder.AddSystem(
                        FlatQuadrotorTrajectorySource(
                            shift_composite_trajectory(traj, -offset)
                        )
                    )
                    _QuadrotorGeometry.AddToBuilder(
                        builder,
                        flat_traj_source.get_output_port(0),
                        scene_graph,
                        f"quadrotor_{i}/{j}",
                    )
            else:
                flat_traj_source = builder.AddSystem(
                    FlatQuadrotorTrajectorySource(traj)
                )
                _QuadrotorGeometry.AddToBuilder(
                    builder,
                    flat_traj_source.get_output_port(0),
                    scene_graph,
                    f"quadrotor_{i}",
                )

            # Visualize the trajectory.
            cartesian_traj_source = builder.AddSystem(TrajectorySource(traj))
            tracer = builder.AddNamedSystem(
                f"quadrotor_trace_{i}",
                TraceVisualizer(
                    meshcat,
                    line_width=self.LINE_WIDTH,
                    rgba=Rgba(*trajectory_colors[i]),
                ),
            )
            meshcat.SetLine(
                f"/drake/traces/persistent_quadrotor_trace_{i}",
                np.hstack(
                    [
                        traj.value(t)
                        for t in np.linspace(0, traj.end_time(), self.NUM_SAMPLES)
                    ]
                ),
                line_width=self.LINE_WIDTH,
                rgba=Rgba(*trajectory_colors[i]),
            )

            meshcat.SetProperty(
                f"/drake/traces/persistent_quadrotor_trace_{i}",
                "visible",
                False,
            )

            builder.Connect(
                cartesian_traj_source.get_output_port(),
                tracer.get_input_port(0),
            )

        diagram = builder.Build()

        # Visualize regions and edges between them.
        for k, region in enumerate(self.all_regions):
            vertices = VPolytope(H=region).vertices()
            meshcat.SetTriangleMesh(
                f"iris/region_{k}",
                vertices,
                ConvexHull(vertices.T).simplices.T,
                self.IRIS_REGION_COLOR,
            )
        meshcat.SetProperty("/drake/iris", "visible", False)

        meshcat.SetLineSegments(
            "edges_between_regions",
            np.vstack(
                [
                    self.all_regions[i].MaximumVolumeInscribedEllipsoid().center()
                    for i, _ in self.edges_between_regions
                ]
            ).T,
            np.vstack(
                [
                    self.all_regions[j].MaximumVolumeInscribedEllipsoid().center()
                    for _, j in self.edges_between_regions
                ]
            ).T,
            line_width=self.LINE_WIDTH,
            rgba=self.EDGES_BETWEEN_REGIONS_COLOR,
        )
        meshcat.SetProperty("/drake/edges_between_regions", "visible", False)

        # Set up a simulator to run this diagram
        simulator = Simulator(diagram)
        if trajectories:
            meshcat.StartRecording()
            simulator.AdvanceTo(total_animation_duration)
            meshcat.PublishRecording()
        else:
            simulator.Initialize()
            diagram.ForcedPublish(simulator.get_context())

    def save(self, filename: str) -> None:
        """Save the environment to an sdf file.

        Args:
            filename: The name of the file to save the environment to.

        Throws:
            RuntimeError: If the environment has not been compiled yet.
        """
        if self.tree is None:
            raise RuntimeError("The environment has not been compiled yet.")
        self.tree.write(filename, pretty_print=True)


class _QuadrotorGeometry(LeafSystem):
    """A copy of the QuadrotorGeometry class from the drake examples, with an additional model_name_prefix argument."""

    def __init__(self, scene_graph: SceneGraph, model_name_prefix: str = "") -> None:
        """Create a _QuadrotorGeometry system.

        Args:
            scene_graph: The SceneGraph to register the quadrotor geometry with.
            model_name_prefix: If multiple quadrotors are being added to the
                scene_graph, this prefix will be added to the model name to
                ensure no name collisions occur in meshcat.
        """
        super().__init__()

        plant = MultibodyPlant(time_step=0.0)
        parser = Parser(plant, scene_graph, model_name_prefix)
        parser.SetAutoRenaming(True)

        model_instance_indices = parser.AddModelsFromUrl(
            "package://drake_models/skydio_2/quadrotor.urdf"
        )
        # Remove collision geometries.  This class is only intended to be used for visualization.
        inspector = scene_graph.model_inspector()
        body = plant.GetBodyByName("base_link", model_instance_indices[0])
        frame_id = plant.GetBodyFrameIdIfExists(body.index())
        geom_ids = inspector.GetGeometries(frame_id, role=Role.kProximity)
        for geom_id in geom_ids:
            scene_graph.RemoveGeometry(plant.get_source_id(), geom_id)

        plant.Finalize()

        body_index = plant.GetBodyIndices(model_instance_indices[0])[0]
        self._source_id = plant.get_source_id()
        self._frame_id = plant.GetBodyFrameIdOrThrow(body_index)

        self.state_input_port = self.DeclareVectorInputPort("state", 12)
        self.DeclareAbstractOutputPort(
            "geometry_pose",
            lambda: Value(FramePoseVector()),
            self.output_geometry_pose,
        )

    def output_geometry_pose(
        self, context: Context, output: Value[FramePoseVector]
    ) -> None:
        """Return the pose of the quadrotor geometry in the world frame."""
        state = self.state_input_port.Eval(context)
        pose = RigidTransform(rpy=RollPitchYaw(state[3:6]), p=state[:3])
        output.get_mutable_value().set_value(self._frame_id, pose)

    @classmethod
    def AddToBuilder(
        cls,
        builder: DiagramBuilder,
        quadrotor_state_port: InputPort,
        scene_graph: SceneGraph,
        model_name_prefix: str = "",
    ) -> None:
        """Create, add, and connect a QuadrotorGeometry system.

        Both the `quadrotor_state.get_system()` and `scene_graph`
        systems must have been added to the given `builder` already.
        The `scene_graph` pointer is not retained by the %QuadrotorGeometry
        system.

        Args:
            builder: The DiagramBuilder containing the quadrotor state and
                scene graph systems.
            quadrotor_state_port: The input port containing the quadrotor state.
            scene_graph: The SceneGraph to register the quadrotor geometry with.
            model_name_prefix: If multiple quadrotors are being added to the
                scene_graph, this prefix will be added to the model name to
                ensure no name collisions occur in meshcat.

        Returns:
            The QuadrotorGeometry system that was added to the builder.
        """
        quadrotor_geometry = builder.AddSystem(cls(scene_graph, model_name_prefix))
        builder.Connect(quadrotor_state_port, quadrotor_geometry.get_input_port(0))
        builder.Connect(
            quadrotor_geometry.get_output_port(0),
            scene_graph.get_source_pose_port(quadrotor_geometry._source_id),
        )


class FlatQuadrotorTrajectorySource(LeafSystem):
    """A source that computes the flat state of a quadrotor."""

    GRAVITY_ACCELERATION = 9.81  # m/s^2

    def __init__(self, trajectory: Trajectory, mass: float = 0.775) -> None:
        """Create flatness inverter source for a quadrotor.

        An implementation of the flatness inverter for quadrotors described in
        D. Mellinger and V. Kumar, "Minimum snap trajectory generation and
        control for quadrotors", Proceedings of the 2011 IEEE International
        Conference on Robotics and Automation (ICRA) , pp. 2520--2525, 2011.

        It takes in a trajectory parametrized as x, y, z, yaw uses the
        differential flatness of quadrotors to compute the desired full state
        of a quadrotor.
        x, y, z, roll, pitch, yaw, xDt, yDt, zDt, rollDt, pitchDt, yawDt

        We assume earth like gravity of 9.81 m/s^2.

        This implementation

        Args:
            trajectory: A trajectory parametrized as x, y, z, yaw describing the
                center of mass. The trajectory should be smooth and at least
                three times differentiable. One can also provide a trajectory
                without yaw, in which case the yaw will be set to zero.
            mass: The mass of the quadrotor in kg.
        """
        super().__init__()
        self.traj = trajectory
        self.traj_has_yaw = self.traj.rows() == 4
        self.m = mass

        self.output_port = self.DeclareVectorOutputPort(
            "state", 12, self.calculate_state
        )

    def calculate_state(self, context: Context, output: BasicVector) -> None:
        """Compute the full-state of the quadrotor."""
        # Evaluate the trajectory and it's derivatives.
        psi = self.traj.value(context.get_time())[3] if self.traj_has_yaw else 0
        psiDt = (
            self.traj.EvalDerivative(context.get_time())[3] if self.traj_has_yaw else 0
        )
        x, y, z = self.traj.value(context.get_time())[:3, 0]
        xDt, yDt, zDt = self.traj.EvalDerivative(context.get_time())[:3, 0]
        xDDt, yDDt, zDDt = self.traj.EvalDerivative(context.get_time(), 2)[:3, 0]
        xDDDt, yDDDt, zDDDt = self.traj.EvalDerivative(context.get_time(), 3)[:3, 0]

        # Equation 6 of the referenced paper.
        t = np.array([xDDt, yDDt, zDDt + self.GRAVITY_ACCELERATION])
        z_b = t / np.linalg.norm(t)

        # Compute the roll and pitch angles.
        x_c = np.array([np.cos(psi), np.sin(psi), 0])
        y_b = np.cross(z_b, x_c)
        y_b = y_b / np.linalg.norm(y_b)
        x_b = np.cross(y_b, z_b)

        R_NB = RotationMatrix(R=np.vstack((x_b, y_b, z_b)))
        phi, theta, _ = -RollPitchYaw(R_NB).vector()

        # Compute the angular velocities.
        u_thrust = self.m * np.linalg.norm(t)
        a_Dt = np.array([xDDDt, yDDDt, zDDDt])
        h_w = (self.m / u_thrust) * (a_Dt.T - (z_b.dot(a_Dt) * z_b))

        p = -h_w.dot(y_b)
        q = h_w.dot(x_b)
        r = psiDt * np.eye(3, 3)[2].dot(z_b)

        # x = [x, y, z, roll, pitch, yaw, xDt, yDt, zDt, rollDt, pitchDt, yawDt]
        output.set_value(np.array([x, y, z, phi, theta, psi, xDt, yDt, zDt, p, q, r]))
