import numpy as np
import sys

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

torch.set_default_tensor_type('torch.DoubleTensor')

class FC(nn.Module):
    def __init__(self, n_inputs=4, n_outputs=1):
        super(FC, self).__init__()
        self.n_inputs  = n_inputs
        self.n_outputs = n_outputs
        self.fc1 = nn.Linear(self.n_inputs, self.n_outputs)
    
    def forward(self, x):
        x = self.fc1(x)
        return x

class FCBIG(nn.Module):
    def __init__(self, n_inputs=4, h_sz=8, n_outputs=1):
        super(FCBIG, self).__init__()
        self.n_inputs  = n_inputs
        self.n_outputs = n_outputs
        self.fc2 = nn.Linear(self.n_inputs, h_sz)
        self.fc3 = nn.Linear(h_sz, self.n_outputs)

    def forward(self, x):
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x

class MLPSMALL(nn.Module):
    def __init__(self, n_inputs=4, h_sz=16, layer_norm=False, n_outputs=1):
        super(MLPSMALL, self).__init__()
        self.n_inputs   = n_inputs
        self.layer_norm = layer_norm
        self.n_outputs  = n_outputs

        self.l1 = nn.Linear(self.n_inputs, h_sz)
        self.ln1 = nn.LayerNorm(h_sz)
        self.tanh1 = torch.tanh
        self.l3 = nn.Linear(h_sz, self.n_outputs)
    
    def forward(self, x):
        x = self.l1(x)
        if self.layer_norm: x = self.ln1(x)
        x = self.tanh1(x)
        x = self.l3(x)
        return x

class MLP(nn.Module):
    def __init__(self, n_inputs=4, h_sz=16, layer_norm=False, n_outputs=1):
        super(MLP, self).__init__()
        self.n_inputs   = n_inputs
        self.layer_norm = layer_norm
        self.n_outputs  = n_outputs

        self.l1 = nn.Linear(self.n_inputs, h_sz)
        self.ln1 = nn.LayerNorm(h_sz)
        self.tanh1 = torch.tanh
        self.l2 = nn.Linear(h_sz, h_sz)
        self.ln2 = nn.LayerNorm(h_sz)
        self.tanh2 = torch.tanh
        self.l3 = nn.Linear(h_sz, self.n_outputs)
    
    def forward(self, x):
        x = self.l1(x)
        if self.layer_norm: x = self.ln1(x)
        x = self.tanh1(x)
        x = self.l2(x)
        if self.layer_norm: x = self.ln2(x)
        x = self.tanh2(x)
        x = self.l3(x)
        return x

class CONV(nn.Module):
    def __init__(self):
        super(CONV, self).__init__()
        self.conv1 = nn.Conv2d(3, 6, 5)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(6, 16, 5)
        self.fc1 = nn.Linear(16 * 5 * 5, 84)
        self.fc2 = nn.Linear(84, 16)
        self.fc3 = nn.Linear(16, 1)

    def forward(self, x):
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        x = x.view(-1, 16 * 5 * 5)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x
