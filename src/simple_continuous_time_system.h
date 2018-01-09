#include "drake/systems/framework/leaf_system.h"

class SimpleContinuousTimeSystem : public drake::systems::LeafSystem<double> {
 public:
  SimpleContinuousTimeSystem() {
    this->DeclareOutputPort(drake::systems::kVectorValued,
                            1,  // dimension of output (y) = 1
                            drake::systems::kContinuousSampling);
  }
  ~SimpleContinuousTimeSystem() override{};

  // xdot = -x + x^3
  void EvalTimeDerivatives(
      const drake::systems::Context<double>& context,
      drake::systems::ContinuousState<double>* derivatives) const override {
    double x = context.get_continuous_state()->get_state().GetAtIndex(0);
    double xdot = -x + std::pow(x, 3.0);
    derivatives->get_mutable_state()->SetAtIndex(0, xdot);
  }

  // y = x
  void EvalOutput(const drake::systems::Context<double>& context,
                  drake::systems::SystemOutput<double>* output) const override {
    double x = context.get_continuous_state()->get_state().GetAtIndex(0);
    output->GetMutableVectorData(0)->SetAtIndex(0, x);
  }

 protected:
  // allocate a basic vector of dimension 1
  std::unique_ptr<drake::systems::ContinuousState<double>>
  AllocateContinuousState() const override {
    return std::make_unique<drake::systems::ContinuousState<double>>(
        std::make_unique<drake::systems::BasicVector<double>>(1));
  }
};
