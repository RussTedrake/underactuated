

def store_transtion(state, action, reward):
    pass

def choose_action(state):
    return action

def learn():
    pass
    # 1) train on episode
    # 2) reset episode data
    # 3) return discounted reward

def discount_and_norm_rewards():
    pass

def build_network():
    # 1) (placeholders)
    # 2) make network
    # 3) fwd prop -> logits -> NLL -> loss -> train


def run_cartpole:
    state = initial_state

    while True:
        a = choose_action(state)
        state, reward = step(a)
        store(a, state, reward)

        if done:
            learn()
