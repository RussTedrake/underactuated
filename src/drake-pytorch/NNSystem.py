import numpy as np
import sys
import torch

from pydrake.all import (
    AbstractValue,
    AutoDiffXd,
    BasicVector, BasicVector_,
    Context,
    LeafSystem, LeafSystem_,
    PortDataType,
)
from pydrake.systems.scalar_conversion import TemplateSystem

def np_hash(np_obj):
    return hash(np_obj.to_string())

@TemplateSystem.define("NNSystem_", T_list=[float, AutoDiffXd])
def NNSystem_(T):

    class Impl(LeafSystem_[T]):
        '''
        Python implementation of a Drake neural network system.
        Powered by PyTorch.

        PyTorch net parameters can be optonally added to Context 
        as a flat list of float or AutoDiffXd.
        Changes to context parameters will be 'synced' to
        network parameters before each EvalOutput().

        The user can set the context parameters to be AutoDiffXd's to calculate
        gradients w.r.t. network parameters.
        '''
        def _construct(self, pytorch_nn_object, declare_params=False, converter=None):
            '''
                pytorch_nn_object: a function(inputs) -> outputs
            '''
            LeafSystem_[T].__init__(self, converter=converter)

            # Getting relavent qtys out of pytorch_nn_object, for port setup.
            self.network = pytorch_nn_object.double()
            param_dims = list(param.size() for param in pytorch_nn_object.parameters())
            self.n_inputs  = param_dims[0][-1]
            self.n_outputs = param_dims[-1][-1]

            # Optionally expose parameters in Context.
            # TODO(rverkuil): Expose bindings for DeclareNumericParameter and use that here.
            self.declare_params = declare_params
            self.params = np.array([])
            if self.declare_params:
                params = np.hstack([param.data.numpy().flatten() for param in self.network.parameters()])
                self.set_params(params)

            # Input Ports
            self.NN_in_input_port = \
                self._DeclareInputPort(
                    "NN_in", PortDataType.kVectorValued, self.n_inputs)

            # Output Ports
            self.NN_out_output_port = \
                self._DeclareVectorOutputPort(
                    "NN_out", BasicVector_[T](self.n_outputs), self.EvalOutput)

        # Necessary for System Scalar Conversion
        def _construct_copy(self, other, converter=None):
            Impl._construct(
                self, other.network, other.declare_params, converter=converter)

        # Currently using these as a stand in until bindings for DeclareNumeriParameter is added.
        def get_params(self):
            return self.params
        def set_params(self, params):
            self.params = params
            self.param_hash = np_hash(self.params)

        def EvalOutput(self, context, output):
            '''
            u (inputs) --> NN => y (outputs)
                           ^
                           |
                   context.p (parameters)
            '''
            drake_in = self.EvalVectorInput(context, 0)
            assert drake_in.size() == self.n_inputs

            # Possibly sync context.parameters -> self.network.parameters
            # TODO: there must be a more intelligent way to detect change in the context,
            # or to install some callback that will do the copy when the context is changed?
            # OnContextChange(), something like that?
            if self.declare_params and np_hash(self.params) != self.param_hash:
                nn_loader(self.get_params(), network)

            # Pack input
            in_vec = np.array([drake_in.GetAtIndex(i) for i in range(self.n_inputs)])

            # Call a helper here to do the heavy lifting.
            # 3 cases:
            #    1) AutoDiffXd, Params in context and AutoDiffXd's  = Can flow derivs of inputs and params.
            #    2) AutoDiffXd, Params are floats or not in context = Can flow derivs of inputs only.
            #    3) Double                                          = No derivs
            if isinstance(in_vec[0], AutoDiffXd):
                if self.declare_params and isinstance(self.get_params()[0], AutoDiffXd):
                    # Make sure derivatives vectors have the same size.
                    assert in_vec[0].derivatives().shape == self.get_params()[0].derivatives().shape
                out_vec = NNInferenceHelper_autodiff(self.network, in_vec, param_vec=self.get_params())
            else:
                out_vec = NNInferenceHelper_double(self.network, in_vec)

            # Pack output
            for j in range(self.n_outputs):
                output.SetAtIndex(j, out_vec[j])

            # Do we even need to return anything here?
            return out_vec[0]

    return Impl


# Default instantiation.
NNSystem = NNSystem_[None]


# Helper function for loading a list of AutoDiffXd parameters into a network.
def nn_loader(param_list, network):
    params_loaded = 0
    for param in network.parameters():
        T_slice = np.array([param_list[i].value() for i in range(
            params_loaded, params_loaded+param.data.nelement())])
        param.data = torch.from_numpy( T_slice.reshape(list(param.data.size())) )
        params_loaded += param.data.nelement() 


def NNInferenceHelper_double(network, in_vec, debug=False):
    # Ensure that all network weights are doubles.
    network = network.double()

    # Process input
    n_inputs = in_vec.shape[0]
    torch_in = torch.tensor(in_vec, dtype=torch.double)
    if debug: print("torch_in: ", torch_in) 

    # Run the forward pass
    torch_out = network.forward(torch_in)
    if debug: print("torch_out: ", torch_out)
    
    # Process output
    n_outputs = torch_out.shape[0]
    out_vec = np.array([torch_out[j].data.numpy() for j in range(n_outputs)])

    return out_vec


# This function is broken out from NNSystem and has this interface because
# having a function like this is super helpful for creating custom MathematicalProgram
# Costs and Cosntraints that use neural networks.
def NNInferenceHelper_autodiff(network, in_vec, param_vec=np.array([]), debug=False):
    # Do forward pass.
    network   = network.double()
    values    = np.array([item.value() for item in in_vec])
    torch_in  = torch.tensor(values, dtype=torch.double, requires_grad=True)
    torch_out = network.forward(torch_in)
    out_vec   = np.array([torch_out[j].data.numpy() for j in range(torch_out.shape[0])])
    if debug: print("torch_in: ", torch_in)
    if debug: print("torch_out: ", torch_out)
    if debug: print("out_vec: ", out_vec)

    # Determine our dimensions.
    n_derivs  = in_vec[0].derivatives().shape[0]
    n_inputs  = in_vec.shape[0]
    n_params  = param_vec.shape[0]
    n_outputs = out_vec.shape[0]

    # Make a jacobian of the (n_inputs x n_derivs) in_vec
    in_deriv_jac = np.array([elem.derivatives() for elem in in_vec]).reshape((n_inputs, n_derivs))

    # Make a jacobian of the (n_param x n_derivs in param_vec)
    param_deriv_jac = np.array([param.derivatives() for param in param_vec]).reshape((n_params, n_derivs))

    # Make an empty accumulator for Neural network (n_outputs vs n_inputs) jacobian
    out_in_jac = np.zeros((n_outputs, n_inputs)).reshape((n_outputs, n_inputs)).astype(np.double)

    # Make an empty accumulator for Neural network (n_outputs vs n_params) jacobian
    out_param_jac = np.zeros((n_outputs, n_params)).reshape((n_outputs, n_params)).astype(np.double)

    # Fill the above two NN-dependent jacobians
    for j in range(n_outputs):
        # Clear out all the existing grads
        network.zero_grad()

        # Since we are using retain_graph to keep non-leaf gradients, and we 
        # are in a for loop, leaf nodes grad will initially be None, and in subsequent
        # iterations, will be not None.
        if torch_in.grad is not None:
            torch_in.grad.zero_()
        
        # Fill the graph with gradients with respect to output y_j.
        # https://discuss.pytorch.org/t/clarification-using-backward-on-non-scalars/1059
        output_selector = torch.zeros(n_outputs, dtype=torch.double)
        output_selector[j] = 1. # Set the output we want a derivative w.r.t. to 1.
        torch_out.backward(output_selector, retain_graph=True)

        # Calculate the contribution to y_j.derivs w.r.t all the inputs.
        out_in_jac[j] = torch_in.grad.numpy()
        assert out_in_jac.dtype == np.double

        # Optionally add contribution of parameter gradients.
        if param_vec.size:
            dy_jdp = np.hstack([
                        (param.grad.numpy().flatten() if param.grad is not None else [0.]*param.data.nelement())
                            for param in network.parameters()])
            assert len(dy_jdp) == n_params
            out_param_jac[j] = dy_jdp

    # Apply chain rule to get all derivatives from inputs -> outputs and (optionally) params -> outputs.
    out_deriv_jac = out_in_jac.dot(in_deriv_jac)
    if debug: print("{} = {} dot {}: ".format(out_deriv_jac, out_in_jac, in_deriv_jac))
    if param_vec.size:
        out_deriv_jac += out_param_jac.dot(param_deriv_jac)

    # Pack into AutoDiffXd's
    out_vec = np.array([AutoDiffXd(out_vec[j], out_deriv_jac[j]) for j in range(n_outputs)])
    if debug: print("out_vec: ", out_vec)

    return out_vec



