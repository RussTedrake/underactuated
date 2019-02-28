from __future__ import print_function, absolute_import

import numpy as np

import pydrake
from pydrake.all import (
    AutoDiffXd, Expression, Variable,
    MathematicalProgram, SolverType, SolutionResult,
    DirectCollocationConstraint, AddDirectCollocationConstraint,
    PiecewisePolynomial,
    DiagramBuilder, SignalLogger, Simulator, VectorSystem,
)

from NNSystemHelper import (
    make_NN_constraint,
)

import sys, os
sys.path.append('..')
from networks import *

num_inputs = 1
num_states = 4

for kNetConstructor in [FC, FCBIG, MLPSMALL, MLP]:
    num_params = sum(tensor.nelement() for tensor in kNetConstructor().parameters())
    total_params = sum((num_inputs, num_states, num_params))
    print("total params: ", total_params, end='\t')
    sys.stdout.flush()

    constraint = make_NN_constraint(kNetConstructor, num_inputs, num_states, num_params)

    import copy
    np.random.seed(1776)
    # Make total_param number of AutoDiffXd's, with (seeded) random values.
    # Set derivatives array to length total_param with only index i set for ith AutoDiff.
    # values = np.random.randn(total_params)
    values = np.ones(total_params)
    def one_hot(i, n_params):
        ret = np.zeros(n_params)
        ret[i] = 1
        return ret

    print("ad pass")
    uxT = np.array([AutoDiffXd(values[i], one_hot(i, total_params)) for i in range(total_params)])
    out = copy.deepcopy(constraint(uxT)[0])
    out_value = out.value()
    out_derivatives = out.derivatives()
    print("ad pass")

    # f     : function(np.array of AutoDiffXd's) -> array of size one of AutoDiffXd
    # x     : np.array of AutoDiffXd at which to calculate finite_difference
    # idx   : Index of AutoDiffXd in x to perturb
    # delta : magnitude of perturbation of AutoDiffXd at index idx of x
    def finite_difference(f, x, idx, delta):
        x_hi = copy.deepcopy(x)
        x_hi[idx] += delta
        x_lo = copy.deepcopy(x)
        x_lo[idx] -= delta
        return ( f(x_hi)[0].value() - f(x_lo)[0].value() ) / (2*delta)

    print("testing", end='')
    sys.stdout.flush()
    for idx in range(total_params):    
        # Do finite difference calculation and compare against gradient
        grad = finite_difference(constraint, uxT, idx, 0.1)
        ref_grad = out_derivatives[idx]
        np.testing.assert_allclose(grad, ref_grad, atol=1e-3)
    print("\t ...GOOD")

