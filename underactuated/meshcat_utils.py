import time
import typing
from functools import partial
from inspect import Parameter, signature

import numpy as np
from ipywidgets.widgets.interaction import (
    _get_min_max_value,
    _yield_abbreviations_for_parameter,
)
from pydrake.geometry import Cylinder, Meshcat, MeshcatVisualizer, Rgba, Sphere
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.plant import MultibodyPlant
from pydrake.solvers import (
    BoundingBoxConstraint,
    MathematicalProgram,
    MathematicalProgramResult,
)
from pydrake.systems.framework import Context, EventStatus, LeafSystem
from pydrake.trajectories import Trajectory

from underactuated import running_as_notebook


# This class is scheduled for removal.  Use meshcat interaction methods instead.
def _interact(meshcat, callback, **kwargs):
    """
    A poor-person's implementation of a functionality like ipywidgets interact
    https://ipywidgets.readthedocs.io/en/latest/examples/Using%20Interact.html#Basic-interact
    """
    values = {}

    # The following code was adapted from https://github.com/jupyter-widgets/ipywidgets/blob/9326d28c474cb6d2b58876d469489a73e98f903a/python/ipywidgets/ipywidgets/widgets/interaction.py  # noqa
    def find_abbreviations(callback, kwargs):
        """
        Find the abbreviations for the given function and kwargs.
        Return (name, abbrev, default) tuples.
        """
        new_kwargs = []
        try:
            sig = signature(callback)
        except (ValueError, TypeError):
            # can't inspect, no info from function; only use kwargs
            return [(key, value, value) for key, value in kwargs.items()]

        for param in sig.parameters.values():
            for name, value, default in _yield_abbreviations_for_parameter(
                param, kwargs
            ):
                if value is Parameter.empty:
                    raise ValueError(
                        "cannot find widget or abbreviation for argument: {!r}".format(
                            name
                        )
                    )
                new_kwargs.append((name, value, default))
        return new_kwargs

    new_kwargs = find_abbreviations(callback, kwargs)

    for name, abbrev, default in new_kwargs:
        kw = {}
        kw["value"] = None if default is Parameter.empty else default
        if isinstance(abbrev, tuple):
            kw["step"] = abbrev[2] if len(abbrev) != 3 else None
            kw["min"], kw["max"], kw["value"] = _get_min_max_value(
                abbrev[0], abbrev[1], **kw
            )
            if kw["step"] is None:
                kw["step"] = 0.1
            meshcat.AddSlider(name, **kw)
        else:
            raise ValueError("This case is not implemented yet")
            # It might be simple.  I just haven't tried!
        values[name] = kw["value"]

    def update_values():
        changed = False
        for name in values:
            v = meshcat.GetSliderValue(name)
            changed |= v != values[name]
            values[name] = v
        return changed

    # Always call it once.
    callback(**values)

    if not running_as_notebook:
        return

    print("Press the 'Stop Interacting' button in Meshcat to continue.")
    meshcat.AddButton("Stop Interacting")
    while meshcat.GetButtonClicks("Stop Interacting") < 1:
        if update_values():
            callback(**values)
        time.sleep(0.1)

    meshcat.DeleteButton("Stop Interacting")
    for name in values:
        meshcat.DeleteSlider(name)


class MeshcatSliders(LeafSystem):
    """
    A system that outputs the values from meshcat sliders.

    An output port is created for each element in the list `slider_names`.
    Each element of `slider_names` must itself be an iterable collection
    (list, tuple, set, ...) of strings, with the names of sliders that have
    *already* been added to Meshcat via Meshcat.AddSlider().

    The same slider may be used in multiple ports.
    """

    def __init__(self, meshcat: Meshcat, slider_names: typing.List[str]):
        LeafSystem.__init__(self)

        self._meshcat = meshcat
        self._sliders = slider_names
        for i, slider_iterable in enumerate(self._sliders):
            port = self.DeclareVectorOutputPort(
                f"slider_group_{i}",
                len(slider_iterable),
                partial(self._DoCalcOutput, port_index=i),
            )
            port.disable_caching_by_default()

    def _DoCalcOutput(self, context, output, port_index):
        for i, slider in enumerate(self._sliders[port_index]):
            output[i] = self._meshcat.GetSliderValue(slider)


class WsgButton(LeafSystem):
    """Adds a button named `Open/Close Gripper` to the meshcat GUI, and registers the Space key to press it. Pressing this button will toggle the value of the output port from a wsg position command corresponding to an open position or a closed position.

    Args:
        meshcat: The meshcat instance in which to register the button.
    """

    def __init__(self, meshcat: Meshcat):
        LeafSystem.__init__(self)
        port = self.DeclareVectorOutputPort("wsg_position", 1, self._DoCalcOutput)
        port.disable_caching_by_default()
        self._meshcat = meshcat
        self._button = "Open/Close Gripper"
        meshcat.AddButton(self._button, "Space")
        print("Press Space to open/close the gripper")

    def __del__(self):
        self._meshcat.DeleteButton(self._button)

    def _DoCalcOutput(self, context, output):
        position = 0.107  # open
        if (self._meshcat.GetButtonClicks(self._button) % 2) == 1:
            position = 0.002  # close
        output.SetAtIndex(0, position)


class StopButton(LeafSystem):
    """Adds a button named `Stop Simulation` to the meshcat GUI, and registers
    the `Escape` key to press it. Pressing this button will terminate the
    simulation.

    Args:
        meshcat: The meshcat instance in which to register the button.
        check_interval: The period at which to check for button presses.
    """

    def __init__(self, meshcat: Meshcat, check_interval: float = 0.1):
        LeafSystem.__init__(self)
        self._meshcat = meshcat
        self._button = "Stop Simulation"

        self.DeclareDiscreteState([0])  # button click count
        self.DeclareInitializationDiscreteUpdateEvent(self._Initialize)
        self.DeclarePeriodicDiscreteUpdateEvent(check_interval, 0, self._CheckButton)

        # Create the button now (rather than at initialization) so that the
        # CheckButton method will work even if Initialize has never been
        # called.
        meshcat.AddButton(self._button, "Escape")

    def __del__(self):
        # TODO(russt): Provide a nicer way to check if the button is currently
        # registered.
        try:
            self._meshcat.DeleteButton(self._button)
        except:
            pass

    def _Initialize(self, context, discrete_state):
        print("Press Escape to stop the simulation")
        discrete_state.set_value([self._meshcat.GetButtonClicks(self._button)])

    def _CheckButton(self, context, discrete_state):
        clicks_at_initialization = context.get_discrete_state().value()[0]
        if self._meshcat.GetButtonClicks(self._button) > clicks_at_initialization:
            self._meshcat.DeleteButton(self._button)
            return EventStatus.ReachedTermination(self, "Termination requested by user")
        return EventStatus.DidNothing()


def AddMeshcatTriad(
    meshcat: Meshcat,
    path: str,
    length: float = 0.25,
    radius: float = 0.01,
    opacity: float = 1.0,
    X_PT: RigidTransform = RigidTransform(),
):
    """Adds an X-Y-Z triad to the meshcat scene.

    Args:
        meshcat: A Meshcat instance.
        path: The Meshcat path on which to attach the triad. Using relative paths will attach the triad to the path's coordinate system.
        length: The length of the axes in meters.
        radius: The radius of the axes in meters.
        opacity: The opacity of the axes in [0, 1].
        X_PT: The pose of the triad relative to the path.
    """
    meshcat.SetTransform(path, X_PT)
    # x-axis
    X_TG = RigidTransform(RotationMatrix.MakeYRotation(np.pi / 2), [length / 2.0, 0, 0])
    meshcat.SetTransform(path + "/x-axis", X_TG)
    meshcat.SetObject(
        path + "/x-axis", Cylinder(radius, length), Rgba(1, 0, 0, opacity)
    )

    # y-axis
    X_TG = RigidTransform(RotationMatrix.MakeXRotation(np.pi / 2), [0, length / 2.0, 0])
    meshcat.SetTransform(path + "/y-axis", X_TG)
    meshcat.SetObject(
        path + "/y-axis", Cylinder(radius, length), Rgba(0, 1, 0, opacity)
    )

    # z-axis
    X_TG = RigidTransform([0, 0, length / 2.0])
    meshcat.SetTransform(path + "/z-axis", X_TG)
    meshcat.SetObject(
        path + "/z-axis", Cylinder(radius, length), Rgba(0, 0, 1, opacity)
    )


def plot_mathematical_program(
    meshcat: Meshcat,
    path: str,
    prog: MathematicalProgram,
    X: np.ndarray,
    Y: np.ndarray,
    result: MathematicalProgramResult = None,
    point_size: float = 0.05,
):
    """Visualize a MathematicalProgram in Meshcat.

    Args:
        meshcat: A Meshcat instance.
        path: The Meshcat path to the visualization.
        prog: A MathematicalProgram instance.
        X: A 2D array of x values.
        Y: A 2D array of y values.
        result: A MathematicalProgramResult instance; if provided then the
            solution is visualized as a point.
        point_size: The size of the solution point.
    """
    assert prog.num_vars() == 2
    assert X.size == Y.size

    X.size
    values = np.vstack((X.reshape(-1), Y.reshape(-1)))
    costs = prog.GetAllCosts()

    # Vectorized multiply for the quadratic form.
    # Z = (D*np.matmul(Q,D)).sum(0).reshape(nx, ny)

    if costs:
        Z = prog.EvalBindingVectorized(costs[0], values)
        for b in costs[1:]:
            Z = Z + prog.EvalBindingVectorized(b, values)

    cv = f"{path}/constraints"
    for binding in prog.GetAllConstraints():
        if isinstance(binding.evaluator(), BoundingBoxConstraint):
            c = binding.evaluator()
            var_indices = [
                int(prog.decision_variable_index()[v.get_id()])
                for v in binding.variables()
            ]
            satisfied = np.array(
                c.CheckSatisfiedVectorized(values[var_indices, :], 0.001)
            ).reshape(1, -1)
            if costs:
                Z[~satisfied] = np.nan

            v = f"{cv}/{type(c).__name__}"
            Zc = np.zeros(Z.shape)
            Zc[satisfied] = np.nan
            meshcat.PlotSurface(
                v,
                X,
                Y,
                Zc.reshape(X.shape),
                rgba=Rgba(1.0, 0.2, 0.2, 1.0),
                wireframe=True,
            )
        else:
            Zc = prog.EvalBindingVectorized(binding, values)
            evaluator = binding.evaluator()
            low = evaluator.lower_bound()
            up = evaluator.upper_bound()
            cvb = f"{cv}/{type(evaluator).__name__}"
            for index in range(Zc.shape[0]):
                # TODO(russt): Plot infeasible points in a different color.
                infeasible = np.logical_or(
                    Zc[index, :] < low[index], Zc[index, :] > up[index]
                )
                meshcat.PlotSurface(
                    f"{cvb}/{index}",
                    X,
                    Y,
                    Zc[index, :].reshape(X.shape),
                    rgba=Rgba(1.0, 0.3, 1.0, 1.0),
                    wireframe=True,
                )

    if costs:
        meshcat.PlotSurface(
            f"{path}/objective",
            X,
            Y,
            Z.reshape(X.shape),
            rgba=Rgba(0.3, 1.0, 0.3, 1.0),
            wireframe=True,
        )

    if result:
        v = f"{path}/solution"
        meshcat.SetObject(v, Sphere(point_size), Rgba(0.3, 1.0, 0.3, 1.0))
        x_solution = result.get_x_val()
        meshcat.SetTransform(
            v,
            RigidTransform([x_solution[0], x_solution[1], result.get_optimal_cost()]),
        )


def PublishPositionTrajectory(
    trajectory: Trajectory,
    root_context: Context,
    plant: MultibodyPlant,
    visualizer: MeshcatVisualizer,
    time_step: float = 1.0 / 33.0,
):
    """
    Publishes an animation to Meshcat of a MultibodyPlant using a trajectory of the plant positions.

    Args:
        trajectory: A Trajectory instance.
        root_context: The root context of the diagram containing plant.
        plant: A MultibodyPlant instance.
        visualizer: A MeshcatVisualizer instance.
        time_step: The time step between published frames.
    """
    plant_context = plant.GetMyContextFromRoot(root_context)
    visualizer_context = visualizer.GetMyContextFromRoot(root_context)

    visualizer.StartRecording(False)

    for t in np.append(
        np.arange(trajectory.start_time(), trajectory.end_time(), time_step),
        trajectory.end_time(),
    ):
        root_context.SetTime(t)
        plant.SetPositions(plant_context, trajectory.value(t))
        visualizer.ForcedPublish(visualizer_context)

    visualizer.StopRecording()
    visualizer.PublishRecording()
