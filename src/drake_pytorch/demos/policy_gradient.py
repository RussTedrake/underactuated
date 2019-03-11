
# Episode Data
states = []
actions = []
rewards = []

def store_transtion(state, action, reward):
    global states, actions, rewards
    states.append(state)
    actions.append(action)
    rewards.append(reward)

def choose_action(state, network):
    action = network(state)
    return action

# In the tensorflow version, goal of this is to 
# (s, a, r) -> weighted NLL, and then training (loop of repeatedly applying grads with some step)
def build_network():
    # 1) (placeholders)
    # 2) make network
    # 3) fwd prop -> logits -> NLL -> loss -> train

def learn():
    global states, actions, rewards
    # 1) train on episode
    

    # 2) reset episode data
    states = []
    actions = []
    rewards = []
    
    # 3) return discounted reward


def discount_and_norm_rewards():
    global rewards
    # Discount...
    discounted_cumulative_rewards = []
    sum = 0
    for i, reward in enumerate(rewards):
        sum += reward * gamma**i
        discounted_cumulative_rewards[i] = sum

    # ...and norm
    discounted_cumulative_rewards -= np.mean(discounted_cumulative_rewards)
    discounted_cumulative_rewards /= np.std(discounted_cumulative_rewards)

    return discounted_cumulative_rewards


def step(action):
    # TODO: get step logic from drake-gym!
    pass

def run_cartpole:
    global states, actions, rewards
    state = initial_state

    while True:
        action = choose_action(state)
        state, reward, done = step(action)
        store(action, state, reward)

        if done:
            learn()
