import numpy as np
import sys

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

# TODO: needed?
torch.set_default_tensor_type('torch.DoubleTensor')

# TODO: Use torch sequential, or whatever it's called?
class FC(nn.Module):
    def __init__(self, n_inputs=4, n_outputs=1):
        super(FC, self).__init__()
        self.fc1 = nn.Linear(n_inputs, n_outputs)
    
    def forward(self, x):
        x = self.fc1(x)
        return x

class FCBIG(nn.Module):
    def __init__(self, n_inputs=4, h_sz=8, n_outputs=1):
        super(FCBIG, self).__init__()
        self.fc2 = nn.Linear(n_inputs, h_sz)
        self.fc3 = nn.Linear(h_sz, n_outputs)

    def forward(self, x):
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x

class MLPSMALL(nn.Module):
    def __init__(self, n_inputs=4, h_sz=16, n_outputs=1):
        super(MLPSMALL, self).__init__()

        self.l1 = nn.Linear(n_inputs, h_sz)
        self.l2 = nn.Linear(h_sz, n_outputs)
    
    def forward(self, x):
        x = self.l1(x)
        x = torch.tanh(x)
        x = self.l2(x)
        return x

class MLP(nn.Module):
    def __init__(self, n_inputs=4, h_sz=16, n_outputs=1):
        super(MLP, self).__init__()

        self.l1 = nn.Linear(n_inputs, h_sz)
        self.l2 = nn.Linear(h_sz, h_sz)
        self.l3 = nn.Linear(h_sz, n_outputs)
    
    def forward(self, x):
        x = self.l1(x)
        x = torch.tanh(x)
        x = self.l2(x)
        x = torch.tanh(x)
        x = self.l3(x)
        return x

