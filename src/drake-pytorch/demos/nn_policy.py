from graphviz import Source
from IPython.display import HTML
import math
import matplotlib.pyplot as plt
import numpy as np
import time

from pydrake.all import (
    DiagramBuilder,
    FindResourceOrThrow,
    MeshcatVisualizer,
    ModelInstanceIndex,
    MultibodyPlant,
    Parser,
    PackageMap,
    SceneGraph,
    SignalLogger,
    Simulator,
    VectorSystem
)

from visualizer import PendulumVisualizer
from underactuated import (
    PlanarRigidBodyVisualizer
)
from pydrake.examples.pendulum import PendulumPlant

from NNSystem import NNSystem


def RenderSystemWithGraphviz(system, output_file="system_view.gz"):
    ''' Renders the Drake system (presumably a diagram,
    otherwise this graph will be fairly trivial) using
    graphviz to a specified file. '''
    string = system.GetGraphvizString()
    src = Source(string)
    print("Rendered system diagram to: ./"+output_file)
    src.render(output_file, view=False)


def NNTestSetupPendulum(network=None, real_time_rate=1.0):
    # Animate the resulting policy.
    builder = DiagramBuilder()
    # tree = RigidBodyTree("/opt/underactuated/src/cartpole/cartpole.urdf",
    #                      FloatingBaseType.kFixed)
    # plant = RigidBodyPlant(tree)
    plant_system = builder.AddSystem(PendulumPlant())


    # TODO(russt): add wrap-around logic to barycentric mesh
    # (so the policy has it, too)
    class WrapTheta(VectorSystem):
        def __init__(self):
            VectorSystem.__init__(self, 2, 2)

        def _DoCalcVectorOutput(self, context, input, state, output):
            output[:] = input
            twoPI = 2.*math.pi
    #         output[1] = output[1] - twoPI * math.floor(output[1] / twoPI)


    wrap = builder.AddSystem(WrapTheta())
    builder.Connect(plant_system.get_output_port(0), wrap.get_input_port(0))
    vi_policy = builder.AddSystem(NNSystem(network))
    # vi_policy = builder.AddSystem(policy)
    builder.Connect(wrap.get_output_port(0), vi_policy.get_input_port(0))
    builder.Connect(vi_policy.get_output_port(0), plant_system.get_input_port(0))

    logger = builder.AddSystem(SignalLogger(2))
    logger._DeclarePeriodicPublish(0.033333, 0.0)
    builder.Connect(plant_system.get_output_port(0), logger.get_input_port(0))

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_publish_every_time_step(False)

    state = simulator.get_mutable_context().get_mutable_continuous_state_vector()
    state.SetFromVector([0., math.pi])


    simulator.StepTo(4.)

    # Visualize the result as a video.
    vis = PendulumVisualizer()
    ani = vis.animate(logger, repeat=True)

    # plt.show()
    # Things added to get visualizations in an ipynb
    plt.close(vis.fig)
    return HTML(ani.to_html5_video())


def NNTestSetupAcrobot(network=None, real_time_rate=1.0):
        '''
        Forward simulate acrobot with a neural network controller.
        '''
        builder = DiagramBuilder()
        scene_graph = builder.AddSystem(SceneGraph())

        # Make Plant
        sdf_file = "assets/acrobot.sdf"
        urdf_file = "assets/acrobot.urdf"
        plant = builder.AddSystem(MultibodyPlant())
        plant.RegisterAsSourceForSceneGraph(scene_graph)
        Parser(plant, scene_graph).AddModelFromFile(sdf_file)
        plant.Finalize(scene_graph)

        assert plant.geometry_source_is_registered()
        builder.Connect(
            plant.get_geometry_poses_output_port(),
            scene_graph.get_source_pose_port(plant.get_source_id()))
        builder.Connect(
            scene_graph.get_query_output_port(),
            plant.get_geometry_query_input_port())

        # Add
        nn_system = NNSystem(network)
        builder.AddSystem(nn_system)

        # NN -> plant
        builder.Connect(nn_system.NN_out_output_port,
                        plant.get_actuation_input_port())
        # plant -> NN
        builder.Connect(plant.get_continuous_state_output_port(),
                        nn_system.NN_in_input_port)

        # Add meshcat visualizer
        meshcat = MeshcatVisualizer(scene_graph)
        builder.AddSystem(meshcat)
        builder.Connect(scene_graph.get_pose_bundle_output_port(),
                        meshcat.GetInputPort("lcm_visualization"))

        # build diagram
        diagram = builder.Build()
        meshcat.load()
        RenderSystemWithGraphviz(diagram)

        # construct simulator
        simulator = Simulator(diagram)

        simulator.set_publish_every_time_step(False)
        #simulator.set_target_realtime_rate(real_time_rate)
        simulator.Initialize()

        sim_duration = 15.
        simulator.StepTo(sim_duration)
        print("Stepping Complete")

