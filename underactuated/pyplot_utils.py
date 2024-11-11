import numpy as np
from IPython.display import HTML, display
from matplotlib.animation import HTMLWriter
from matplotlib.widgets import Slider
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import Context, LeafSystem
from pydrake.systems.pyplot_visualizer import PyPlotVisualizer
from pydrake.trajectories import Trajectory

from underactuated.jupyter import running_as_notebook


class HistogramVisualizer(PyPlotVisualizer):
    """A simple visualizer that plots a histogram of the input vector."""

    def __init__(
        self, num_samples, bins, xlim, ylim, draw_time_step, figsize=(6, 4), show=True
    ):
        PyPlotVisualizer.__init__(self, draw_time_step, figsize=figsize, show=show)
        self.DeclareVectorInputPort(f"x", num_samples)
        self.num_samples = num_samples
        self.bins = bins
        self.data = [0] * num_samples
        self.scale = 10
        self.limits = xlim
        self.ax.set_xlim(xlim)
        self.ax.axis("auto")
        self.ax.set_ylim(ylim)
        self.patches = None

    def draw(self, context):
        if self.patches:
            [p.remove() for p in self.patches]
        self.data = self.EvalVectorInput(context, 0).value()
        count, bins, self.patches = self.ax.hist(
            self.data,
            bins=self.bins,
            range=self.limits,
            density=False,
            weights=[self.scale / self.num_samples] * self.num_samples,
            facecolor="b",
        )
        self.ax.set_title("t = " + str(context.get_time()))


class SliderSystem(LeafSystem):
    def __init__(self, ax, title, min, max):
        # 0 inputs, 1 output.
        LeafSystem.__init__(self)
        self.DeclareVectorOutputPort("slider", 1, self.DoCalcVectorOutput)
        self.value = 0
        self.slider = Slider(ax, title, min, max, valinit=self.value)
        self.slider.on_changed(self.update)

    def update(self, val):
        self.value = val

    def DoCalcVectorOutput(self, context, output):
        output.SetAtIndex(0, self.value)


def AdvanceToAndVisualize(
    simulator,
    visualizer,
    time,
    time_if_running_headless=0.1,
    movie_filename=None,
):
    """
    Helper to support visualizing a simulation with pyplot visualizer.
    Will simply simulate (with target_realtime_rate = 1) if visualizer.show =
    True, or will record and render an animation if visualizer.show = False. If
    specified, time_if_running_headless will be used instead of time if
    running_as_notebook is False.
    """
    if visualizer._show:
        target_rate = simulator.get_target_realtime_rate()
        simulator.set_target_realtime_rate(1.0)
    else:
        print("simulating... ", end=" ")
        visualizer.start_recording()
    if time_if_running_headless and not running_as_notebook:
        time = time_if_running_headless
    simulator.AdvanceTo(time)
    if not visualizer._show or movie_filename:
        print("done.\ngenerating animation...")
        ani = visualizer.get_recording_as_animation()
        display(HTML(ani.to_jshtml()))
        if movie_filename:
            with open(movie_filename, "w") as f:
                f.write(ani.to_jshtml())
    else:
        simulator.set_target_realtime_rate(target_rate)


def AdvanceToAndSaveAnimation(simulator, visualizer, time, filename):
    visualizer.start_recording()
    simulator.AdvanceTo(time)
    ani = visualizer.get_recording_as_animation()
    # Note: Wanted to use embed_frames=True, but it did not render
    # the image properly for me.
    writer = HTMLWriter()
    writer.frame_format = "svg"
    ani.save(filename, writer=writer)


def AnimatePositionTrajectory(
    trajectory: Trajectory,
    root_context: Context,
    plant: MultibodyPlant,
    visualizer: PyPlotVisualizer,
    time_step: float = 1.0 / 33.0,
):
    """
    Returns an animation of a MultibodyPlant using a trajectory of the plant positions.

    Args:
        trajectory: A Trajectory instance.
        root_context: The root context of the diagram containing plant.
        plant: A MultibodyPlant instance.
        visualizer: A PyPlotVisualizer instance.
        time_step: The time step between published frames.
    """
    plant_context = plant.GetMyContextFromRoot(root_context)
    visualizer_context = visualizer.GetMyContextFromRoot(root_context)

    visualizer.start_recording()

    for t in np.append(
        np.arange(trajectory.start_time(), trajectory.end_time(), time_step),
        trajectory.end_time(),
    ):
        root_context.SetTime(t)
        plant.SetPositions(plant_context, trajectory.value(t)[: plant.num_positions()])
        visualizer.ForcedPublish(visualizer_context)

    visualizer.stop_recording()
    return visualizer.get_recording_as_animation()
