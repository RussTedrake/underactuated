import numpy as np


class Adam():

    def __init__(self,
                 params,
                 lr=1e-3,
                 betas=(0.9, 0.999),
                 eps=1e-8,
                 weight_decay=0,
                 amsgrad=False):
        self.params = params
        if not 0.0 <= lr:
            raise ValueError("Invalid learning rate: {}".format(lr))
        self.lr = lr
        if not 0.0 <= eps:
            raise ValueError("Invalid epsilon value: {}".format(eps))
        self.eps = eps
        if not 0.0 <= betas[0] < 1.0:
            raise ValueError("Invalid beta parameter at index 0: {}".format(
                betas[0]))
        if not 0.0 <= betas[1] < 1.0:
            raise ValueError("Invalid beta parameter at index 1: {}".format(
                betas[1]))
        self.beta1 = betas[0]
        self.beta2 = betas[1]
        if not 0.0 <= weight_decay:
            raise ValueError(
                "Invalid weight_decay value: {}".format(weight_decay))
        self.weight_decay = weight_decay
        self.amsgrad = amsgrad

        self.exp_avgs = 0 * params
        self.exp_avgs_sqs = 0 * params
        self.max_exp_avgs_sqs = 0 * params
        self.num_steps = 0

    def step(self, loss, dloss_dparams):
        if self.weight_decay != 0.0:
            dloss_dparams += self.weight_decay * params

        self.num_steps += 1

        bias_correction1 = 1 - self.beta1**self.num_steps
        bias_correction2 = 1 - self.beta2**self.num_steps

        self.exp_avgs *= self.beta1
        self.exp_avgs += (1 - self.beta1) * dloss_dparams

        self.exp_avgs_sqs *= self.beta2
        self.exp_avgs_sqs += (1 - self.beta2) * (dloss_dparams**2)

        if self.amsgrad:
            # Maintains the maximum of all 2nd moment running avg. till now
            np.maximumtorch.maximum(self.max_exp_avg_sqs,
                                    exp_avgs_sqs,
                                    out=self.max_exp_avgs_sqs)
            # Use the max. for normalizing running avg. of gradient
            denom = (np.sqrt(self.max_exp_avgs_sqs)
                     / np.sqrt(bias_correction2)) + self.eps
        else:
            denom = (np.sqrt(self.exp_avgs_sqs)
                     / np.sqrt(bias_correction2)) + self.eps
        step_size = self.lr / bias_correction1
        self.params -= step_size * self.exp_avgs / denom
