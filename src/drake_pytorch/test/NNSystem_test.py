from __future__ import print_function, absolute_import

import copy
import numpy as np

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

import pydrake
import pydrake.autodiffutils
from pydrake.autodiffutils import AutoDiffXd
from pydrake.all import (
    AutoDiffXd, Expression, Variable,
    MathematicalProgram, SolverType, SolutionResult,
    DirectCollocationConstraint, AddDirectCollocationConstraint,
    PiecewisePolynomial,
    DiagramBuilder, SignalLogger, Simulator, VectorSystem,
)

from NNSystem import (
    NNSystem, NNSystem_,
    nn_inference_double,
    nn_inference_autodiff,
    nn_loader
)
from networks import FC, FCBIG, MLPSMALL, MLP

torch.manual_seed(1)

# Test 1
# Try creating a simple NNSystem and show that it supports AutoDiffXd's.
# Define network, input
torch.set_default_tensor_type('torch.DoubleTensor')
net = FC(n_inputs=2, n_outputs=1)
print("net.parameters()= ", list(net.parameters()))
autodiff_in = np.array([
    [AutoDiffXd(1., [1., 0.]), ],
    [AutoDiffXd(1., [0., 1.]), ]
])

# Make system and give input
nn_system = NNSystem_[AutoDiffXd](pytorch_nn_object=net)
context = nn_system.CreateDefaultContext()
context.FixInputPort(0, autodiff_in)

# Allocate and Eval output
output = nn_system.AllocateOutput()
nn_system.CalcOutput(context, output)
autodiff = output.get_vector_data(0).CopyToVector()[0]
print("autodiff: ", autodiff)
print("derivatives: ", autodiff.derivatives())
np.testing.assert_almost_equal(autodiff.value(), -1.3861268635031063)
np.testing.assert_allclose(
        autodiff.derivatives(), [-0.620764, -0.389539], atol=1e-3)
print()


# Test 2
# Check that only difference between nn_inference_double
# and nn_inference_autodiff is zero gradients.
network = FCBIG(2)
in_vec_double = np.array([1., 1.])
in_vec_autodiff = np.array([
    AutoDiffXd(1., [1., 0.]),
    AutoDiffXd(1., [0., 1.])
])
out_vec_double = nn_inference_double(network, in_vec_double)
out_vec_autodiff = nn_inference_autodiff(network, in_vec_autodiff)[0]
np.testing.assert_allclose(
        out_vec_double, [elem.value() for elem in out_vec_autodiff])


def finite_difference_check_autodiffs(autodiff_params=False, debug=False):
    print("autodiff_params: ", autodiff_params)
    NUM_INPUTS = 3
    NUM_OUTPUTS = 3
    NETS_TO_TEST = [
                    FC,
                    FCBIG,
                    MLPSMALL,
                   ]

    for kNetConstructor in NETS_TO_TEST:
        print(kNetConstructor.__name__, end='')
        if debug:
            np.random.seed(1)
            torch.manual_seed(1)

        # Make a net and calculate n_inputs and n_outputs
        network = kNetConstructor(n_inputs=NUM_INPUTS, n_outputs=NUM_OUTPUTS)
        param_dims = list(param.size() for param in network.parameters())
        n_params = np.sum(np.prod(param_dim) for param_dim in param_dims)
        n_inputs = param_dims[0][-1]
        n_outputs = param_dims[-1][-1]
        total_params = n_inputs + n_params if autodiff_params else n_inputs
        param_vec = np.array([])

        def one_hot(i, N):
            ret = np.zeros(N)
            ret[i] = 1
            return ret

        if debug:
            for param in network.parameters():
                torch.nn.init.uniform_(param.data, 1, 1)

        # Make total_param number of AutoDiffXd's, with (seeded) random values.
        # Set derivatives array to length total_param with only index i set for
        # ith AutoDiff.
        values = (np.ones(n_inputs) if debug else np.random.randn(n_inputs))
        in_vec = np.array(
                [AutoDiffXd(values[i], one_hot(i, total_params))
                    for i in range(n_inputs)])
        if autodiff_params:
            values = np.hstack(param.clone().detach().numpy().flatten()
                               for param in network.parameters())
            param_vec = np.array(
                    [AutoDiffXd(values[i], one_hot(n_inputs+i, total_params))
                        for i in range(n_params)])
            nn_loader(param_vec, network)

        # First, generate all the AutoDiffXd outputs.
        out_vec = nn_inference_autodiff(network,
                                        in_vec,
                                        param_vec=param_vec,
                                        debug=debug)[0]
        if debug:
            print("param grads: ",
                  [param.grad for param in network.parameters()])

        # f     : function(np.array of AutoDiffXd's) -> array of one AutoDiffXd
        # x     : np.array of AutoDiffXd's
        # idx   : Index of AutoDiffXd in x to perturb
        # delta : magnitude of perturbation of AutoDiffXd at index idx of x
        def finite_difference(f, x, idx, delta=1e-7):
            x_hi = copy.deepcopy(x)
            x_hi[idx] += delta
            x_lo = copy.deepcopy(x)
            x_lo[idx] -= delta
            out_hi = np.array([elem.value() for elem in f(x_hi)])
            out_lo = np.array([elem.value() for elem in f(x_lo)])
            return (out_hi - out_lo) / (2*delta)

        # AutoDiff method returns a list of AutoDiffXd's by output.  It is like
        # a Jacobian matrix that is n_output x n_input.
        # Repeatedly finite differencing every input will return me vectors of
        # dervivates at each output.  This is like a Jacobian that is n_input x
        # n_output.  Let's take both jacobians, transpose one, and compare them
        # for equality.

        # Jacobian will be of size total_params**2.
        # Let's not let that get too big.
        assert total_params < 1e3

        # Make AutoDiffXd Jacobian, (n_output x n_input)
        ad_hess = np.array([elem.derivatives() for elem in out_vec])

        # Make Finite Difference Jacobian, (n_input x n_output)
        def fn(x):
            # Wrapper for nn_inference_autodiff that accepts a single list of
            # gradients and assigns first n_inputs to in_list, and all the rest
            # to param_list.  Also creates a fresh NN using param_list.
            in_vec = x[:n_inputs]
            param_vec = x[n_inputs:]
            if param_vec.size:
                nn_loader(param_vec, network)
            return nn_inference_autodiff(network,
                                         in_vec,
                                         param_vec=param_vec,
                                         debug=False)[0]
        fd_hess = np.array([finite_difference(
                    fn,
                    np.hstack((in_vec, param_vec)), inp_idx)
                    for inp_idx in range(total_params)])

        # Do our comparison.
        if False:
            print('fd_hess.T: ', fd_hess.T)
            print('ad_hess:   ', ad_hess)
        np.testing.assert_allclose(fd_hess.T, ad_hess, atol=1e-3)
        print("\t ...GOOD")


# Test 3
# Use finite difference method to test AutoDiff flow from input -> output.
finite_difference_check_autodiffs(autodiff_params=False)


# Test 4
# Use finite difference method to test AutoDiff flow from param -> output.
finite_difference_check_autodiffs(autodiff_params=True, debug=False)
