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
    PortDataType,
)
from pydrake.systems.scalar_conversion import TemplateSystem

# Import all the networks definitions we've made in the other file.
import sys, os
sys.path.append('..')
from networks import *
from NNSystem import NNSystem

torch.set_default_tensor_type('torch.DoubleTensor')



# TODO: use this in place of create_nn_system thing in traj.vis
def create_nn(kNetConstructor, params_list):
    # Construct a model with params T
    net = kNetConstructor()
    params_loaded = 0
    for param in net.parameters():
        param_slice = np.array([params_list[i] for i in range(params_loaded, params_loaded+param.data.nelement())])
        param.data = torch.from_numpy(param_slice.reshape(list(param.data.size())))
        params_loaded += param.data.nelement() 

    return net

def create_nn_policy_system(kNetConstructor, params_list):
    net = create_nn(kNetConstructor, params_list)
    nn_policy = NNSystem(pytorch_nn_object=net)
    return nn_policy

import copy
def make_NN_constraint_old(kNetConstructor, num_inputs, num_states, num_params, do_asserts=False):
    def constraint(uxT):
        # start = time.time()
    # ##############################
    # #    JUST FOR DEBUGGING!
    # ##############################
    #     for elem in uxT:
    #         print(elem.derivatives())
    #     print()
    #     return uxT
    # prog.AddConstraint(constraint, -np.array([.1]*5), np.array([.1]*5), np.hstack((prog.input(0), prog.state(0))))
    # ##############################
        # Force use of AutoDiff Values, so that EvalBinding works (it normally only uses doubles...)
        double_ver = False
        if uxT.dtype != np.object:
            double_ver = True
            uxT = copy.deepcopy(uxT)
            def one_hot(i):
                ret = np.zeros(len(uxT))
                ret[i] = 1
                return ret
            uxT = np.array([AutoDiffXd(val, one_hot(i)) for i, val in enumerate(uxT)])

        u = uxT[:num_inputs]
        x = uxT[num_inputs:num_inputs+num_states]
        T = uxT[num_inputs+num_states:]
        
        # We assume that .derivative() at index i 
        # of uxT is a one hot with only derivatives(i) set. Check this here.
        n_derivatives = len(u[0].derivatives())
        if do_asserts: assert n_derivatives == sum((num_inputs, num_states, num_params))
        for i, elem in enumerate(uxT): # TODO: is uxT iterable without any reshaping?
            if do_asserts: assert n_derivatives == len(elem.derivatives())
            one_hot = np.zeros((n_derivatives))
            one_hot[i] = 1
            if do_asserts: np.testing.assert_array_equal(elem.derivatives(), one_hot)
        
        # Construct a model with params T
        net = kNetConstructor()
        params_loaded = 0
        for param in net.parameters():
            T_slice = np.array([T[i].value() for i in range(params_loaded, params_loaded+param.data.nelement())])
            param.data = torch.from_numpy(T_slice.reshape(list(param.data.size())))
            params_loaded += param.data.nelement() 
            
        # Do forward pass.
        x_values = np.array([elem.value() for elem in x])
        torch_in = torch.tensor(x_values, requires_grad=True)
        torch_out = net.forward(torch_in)
        y_values = torch_out.clone().detach().numpy()
        
        # Calculate all gradients w.r.t. single output.
        # WARNING: ONLY WORKS IF NET HAS ONLY ONE OUTPUT.
        torch_out.backward(retain_graph=True)
        
        y_derivatives = np.zeros(n_derivatives)
        y_derivatives[:num_inputs] = np.zeros(num_inputs) # No gradient w.r.t. inputs yet.
        y_derivatives[num_inputs:num_inputs+num_states] = torch_in.grad
        T_derivatives = []
        for param in net.parameters():
            if param.grad is None:
                T_derivatives.append( [0.]*param.data.nelement() )
            else:
                if do_asserts: assert param.data.nelement() == param.grad.nelement()
                if do_asserts: np.testing.assert_array_equal( list(param.data.size()), list(param.grad.size()) )
                T_derivatives.append( param.grad.numpy().flatten() ) # Flatten will return a copy.
        if do_asserts: assert np.hstack(T_derivatives).size == num_params
        y_derivatives[num_inputs+num_states:] =  np.hstack(T_derivatives)
        
        y = AutoDiffXd(y_values, y_derivatives)
        
        # constraint is Pi(x) == u
        # end = time.time()
        # print("constraint eval: ", end - start)
        if double_ver:
            ret = (u-y)[0].value()
            #print("ret: ", ret)
            return [ret]
        return u - y
    return constraint


def make_NN_constraint(kNetConstructor, num_inputs, num_states, num_params, do_asserts=False):
    def constraint(uxT):
        # Force use of AutoDiff Values, so that EvalBinding works (it normally only uses doubles...)
        double_ver = False
        if uxT.dtype != np.object:
            double_ver = True
            uxT = copy.deepcopy(uxT)
            def one_hot(i):
                ret = np.zeros(len(uxT))
                ret[i] = 1
                return ret
            uxT = np.array([AutoDiffXd(val, one_hot(i)) for i, val in enumerate(uxT)])

        u = uxT[:num_inputs]
        x = uxT[num_inputs:num_inputs+num_states]
        T = uxT[num_inputs+num_states:]
        
        # We assume that .derivative() at index i 
        # of uxT is a one hot with only derivatives(i) set. Check this here.
        n_derivatives = len(u[0].derivatives())
        if do_asserts: assert n_derivatives == sum((num_inputs, num_states, num_params))
        for i, elem in enumerate(uxT): # TODO: is uxT iterable without any reshaping?
            if do_asserts: assert n_derivatives == len(elem.derivatives())
            one_hot = np.zeros((n_derivatives))
            one_hot[i] = 1
            if do_asserts: np.testing.assert_array_equal(elem.derivatives(), one_hot)
        
        # Construct a model with params T
        net = kNetConstructor()
        params_loaded = 0
        for param in net.parameters():
            T_slice = np.array([T[i].value() for i in range(params_loaded, params_loaded+param.data.nelement())])
            param.data = torch.from_numpy(T_slice.reshape(list(param.data.size())))
            params_loaded += param.data.nelement() 
            
        # Do forward pass.
        x_values = np.array([elem.value() for elem in x])
        torch_in = torch.tensor(x_values, requires_grad=True)
        torch_out = net.forward(torch_in)
        y_values = torch_out.clone().detach().numpy()
        
        # Calculate all gradients w.r.t. single output.
        # WARNING: ONLY WORKS IF NET HAS ONLY ONE OUTPUT.
        torch_out.backward(retain_graph=True)
        
        y_derivatives = np.zeros(n_derivatives)
        y_derivatives[:num_inputs] = np.zeros(num_inputs) # No gradient w.r.t. inputs yet.
        y_derivatives[num_inputs:num_inputs+num_states] = torch_in.grad
        T_derivatives = []
        for param in net.parameters():
            if param.grad is None:
                T_derivatives.append( [0.]*param.data.nelement() )
            else:
                if do_asserts: assert param.data.nelement() == param.grad.nelement()
                if do_asserts: np.testing.assert_array_equal( list(param.data.size()), list(param.grad.size()) )
                T_derivatives.append( param.grad.numpy().flatten() ) # Flatten will return a copy.
        if do_asserts: assert np.hstack(T_derivatives).size == num_params
        y_derivatives[num_inputs+num_states:] =  np.hstack(T_derivatives)
        
        y = AutoDiffXd(y_values, y_derivatives)
        
        # constraint is Pi(x) == u
        # end = time.time()
        # print("constraint eval: ", end - start)
        if double_ver:
            ret = (u-y)[0].value()
            #print("ret: ", ret)
            return [ret]
        return u - y
    return constraint
