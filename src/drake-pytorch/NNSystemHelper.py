import copy
import os
import numpy as np
import sys
import time

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

from pydrake.all import (
    AutoDiffXd,
    BasicVector, BasicVector_,
    LeafSystem, LeafSystem_,
    PortDataType
)
from pydrake.systems.scalar_conversion import TemplateSystem

# Import all the networks definitions we've made in the other file.
from networks import *
from NNSystem import NNSystem, NNInferenceHelper_autodiff

torch.set_default_tensor_type('torch.DoubleTensor')

# TODO:
# 1) get rid of set default tensor type to double?
# Get rid of assert statements?

# Return a one-hot vector of length n, with index i set.
def one_hot(n, i):
    ret = np.zeros(len(uxT))
    ret[i] = 1
    return ret

# Create a network
# @param kNetConstructor: Constructor for a PyTorch Neural Network
# @param params_list: double or AutoDiffXd list of parameter values
def create_nn(kNetConstructor, params_list):
    # Construct a model with params T
    net = kNetConstructor()
    params_loaded = 0
    for param in net.parameters():
        #param_slice = np.array([params_list[i] for i in range(params_loaded, params_loaded+param.data.nelement())])
        param_slice = np.array(params_list[params_loaded : params_loaded+param.data.nelement()])
        param_slice = param_slice.reshape(list(param.data.size()))
        param.data = torch.from_numpy(param_slice)
        params_loaded += param.data.nelement()

    return net

# This function, makes a custom constraint that can be given to a Drake MathematicalProgram.
def make_NN_constraint(kNetConstructor, num_inputs, num_states, num_params, debug=False):
    def constraint(uxT):
        # Force use of AutoDiff Values, so that EvalBinding works (it normally only uses doubles...)
        using_double = uxT.dtype != np.object
        if using_double:
            #uxT = copy.deepcopy(uxT)
            uxT = np.array([AutoDiffXd(val, one_hot(len(uxT), i)) for i, val in enumerate(uxT)])

        u = uxT[:num_inputs]
        x = uxT[num_inputs:num_inputs+num_states]
        T = uxT[num_inputs+num_states:]
        
        # Construct a model with params T
        params_list = np.array([param.value() for param in T])
        network = create_nn(kNetConstructor, params_list)
        out_vec, out_in_jac, out_param_jac = NNInferenceHelper_autodiff(network, x, param_vec=T, debug=debug)
           
        # Create output
        y_values = np.array([elem.value() for elem in out_vec])
        y_derivatives = np.hstack( [np.zeros(num_inputs), out_in_jac.flatten(), out_param_jac.flatten()]) # No gradient w.r.t. inputs yet.
        y = AutoDiffXd(y_values, y_derivatives)
        
        # constraint is Pi(x) == u
        ret = u - y

        # Return scalar if in double mode.
        if using_double:
            ret = [ret[0].value()]

        return ret
    return constraint


