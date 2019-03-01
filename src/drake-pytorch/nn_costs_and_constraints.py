from __future__ import print_function, absolute_import

# Standard
import copy
import os
import math
import sys
import time

# 3rd Party
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

# PyDrake
from pydrake.all import (
    AutoDiffXd,
    DirectCollocation, 
    PiecewisePolynomial, 
    SignalLogger,
    Simulator,
    SolutionResult,
)
from pydrake.examples.pendulum import PendulumPlant

# Local
from networks import *
from NNSystem import NNSystem, NNInferenceHelper_autodiff
from visualizer import PendulumVisualizer # TODO: get rid of the copy of visualizer


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
        param_slice = np.array(params_list[params_loaded : params_loaded+param.data.nelement()]) \
                .reshape(list(param.data.size()))
        param.data = torch.from_numpy(param_slice)
        params_loaded += param.data.nelement()

    return net

# This function, makes a custom constraint that can be given to a Drake MathematicalProgram.
def make_NN_constraint(kNetConstructor, num_inputs, num_states, num_params, debug=False):
    def constraint(uxT):
        # Force use of AutoDiff Values, so that EvalBinding works (it normally only uses doubles...)
        using_double = uxT.dtype != np.object
        if using_double:
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



vis_cb_counter = 0
def make_pendulum_dircol_prog():
    plant = PendulumPlant()
    context = plant.CreateDefaultContext()
    dircol = DirectCollocation(plant, context,
                               num_time_samples=32,
                               minimum_timestep=0.002,
                               maximum_timestep=0.25)

    # Time Constraints
    dircol.AddEqualTimeIntervalsConstraints()
    #dircol.AddFinalCost(dircol.time())

    # Input Constraints
    torque_limit = 5.
    u = dircol.input()
    dircol.AddConstraintToAllKnotPoints(-torque_limit <= u[0])
    dircol.AddConstraintToAllKnotPoints(u[0] <= torque_limit)

    # State Constraints
    initial_state = (-1., 0.)
    dircol.AddBoundingBoxConstraint(initial_state, initial_state,
                                    dircol.initial_state())
    final_state = (math.pi, 0.)
    dircol.AddBoundingBoxConstraint(final_state, final_state,
                                    dircol.final_state())

    # Cost function
    u = dircol.input()
    x = dircol.state()
    x_diff = x - np.array(final_state)
    dircol.AddRunningCost( 2*x_diff.dot(x_diff) + 25*u.dot(u) )

    # Warm Start (Linear)
    initial_u_trajectory = PiecewisePolynomial()
    initial_x_trajectory = \
        PiecewisePolynomial.FirstOrderHold([0., 4.],
                                       np.column_stack((initial_state,
                                                        final_state)))
    dircol.SetInitialTrajectory(initial_u_trajectory, initial_x_trajectory)

    # Visualiation
    def vis(sample_times, values):
        global vis_cb_counter
        vis_cb_counter += 1
        if vis_cb_counter % 10 != 0:
            return

        plt.figure()
        plt.title('x vs x_dot')
        plt.xlabel('x')
        plt.ylabel('x_dot')

        x, x_dot = values[0], values[1]
        plt.plot(x, x_dot, '-o', label=vis_cb_counter)
        plt.show()
    dircol.AddStateTrajectoryCallback(vis)

    return dircol


def make_constrained_opt(kNetConstructor):
    dircol = make_pendulum_dircol_prog()
    num_samples = len(dircol.time())
    num_inputs  = len(dircol.input())
    num_states  = len(dircol.state())
    
    # Create a network
    # Determine num_params and add them to the prog.
    dummy_net = kNetConstructor()
    num_params = sum(tensor.nelement() for tensor in dummy_net.parameters())
    T = dircol.NewContinuousVariables(num_params, 'T')

    # Very important - preload t with the net's initialization.
    # default zero initialization will give you zero gradients!!!!
    params_loaded = 0
    initial_guess = [AutoDiffXd]*num_params
    for param in dummy_net.parameters(): # Here's where we make a dummy net. Let's seed this?
        param_values = param.data.numpy().flatten()
        for i in range(param.data.nelement()):
            initial_guess[params_loaded + i] = param_values[i]
        params_loaded += param.data.nelement()
    dircol.SetInitialGuess(T, np.array(initial_guess))

    for i in range(num_samples):
        # Now let's add on the policy deviation cost... 
        constraint = make_NN_constraint(kNetConstructor, num_inputs, num_states, num_params)
        lb         = np.array([-.0001])
        ub         = np.array([.0001])
        var_list   = np.hstack((dircol.input(i), dircol.state(i), T))
        dircol.AddConstraint(lambda x: [constraint(x)], lb, ub, var_list)
        #dircol.AddCost(lambda x: constraint(x)[0]**2, var_list)

    return dircol

