
#include "simple_continuous_time_system.h"

#include "drake/systems/analysis/simulator.h"

int main(int argc, char* argv[]) {
  // create the simple system
  SimpleContinuousTimeSystem system;

  // create the simulator
  drake::systems::Simulator<double> simulator(system);

  // set the initial conditions x(0);
  auto initial_conditions = simulator.get_mutable_context();
  initial_conditions->get_mutable_continuous_state()
      ->get_mutable_state()
      ->SetAtIndex(0, .9);

  // simulate for 10 seconds
  simulator.StepTo(10);

  // make sure the simulation converges to the stable fixed point at x=0
  DRAKE_ASSERT(std::abs(simulator.get_context()
                            .get_continuous_state()
                            ->get_state().GetAtIndex(0)) < 1.0e-4);

  // TODO(russt): make a plot of the resulting trajectory (using vtk?)

  return 0;
}
