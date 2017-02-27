import numpy as np

DISCOUNT_THRESHOLD = 0.01


class Critic:
    def __init__(self, state_dimensions, number_of_actions, time_horizon, discount, buffer_size):
        self.rewards = np.zeros(state_dimensions)
        self.timeHorizon = time_horizon
        self.discount = discount
        self.number_of_Actions = number_of_actions

        self.bufferSize = buffer_size
        state_action_dims = state_dimensions + (number_of_actions,)
        action_knowledge_dims = state_action_dims + (len(state_dimensions),)
        self.actionKnowledge = np.zeros(action_knowledge_dims, dtype=np.int_)
        action_knowledge_buffer_dims = state_action_dims + (len(state_dimensions) + 1, buffer_size)
        self.actionKnowledgeBuffer = np.zeros(action_knowledge_buffer_dims, dtype=np.int_)
        self.bufferIndex = np.zeros(state_action_dims, dtype=np.int_)

    def update_action_knowledge(self, previous_state, previous_action, state):
        state_action = previous_state + (previous_action,)
        new_knowledge = np.asarray(state).T
        number_in_buffer = self.bufferIndex[state_action]

        observation_location = -1
        # check for next state in buffer
        for i in range(0, number_in_buffer):
            index = state_action + (slice(1, 3), i)
            if np.array_equal(self.actionKnowledgeBuffer[index], new_knowledge):
                observation_location = i

        if observation_location < 0:
            if number_in_buffer < self.bufferSize:
                self.actionKnowledgeBuffer[state_action + (slice(1, 3), number_in_buffer)] = new_knowledge
                self.actionKnowledgeBuffer[state_action + (0, number_in_buffer)] += 1
                self.bufferIndex[state_action] += 1
        else:
            self.actionKnowledgeBuffer[state_action + (0, observation_location)] += 1

        mode_index = np.argmax(self.actionKnowledgeBuffer[state_action + (0, Ellipsis)])
        mode_action = self.actionKnowledgeBuffer[state_action + (slice(1, 3), mode_index)]

        self.actionKnowledge[state_action] = mode_action

    def get_td_error(self, previous_state, new_state, policy, action_knowledge):
        r1 = self.rewards[new_state]
        # print "Reward of new state: ", r1
        v1 = self.value(new_state, policy)
        # print "Value of new state: ", v1
        v0 = self.value(previous_state, policy)
        # print "Value of previous state: ", v0
        return r1 + self.discount * v1 - v0

    def value(self, state, policy):
        return value_recursion(state, 0, policy, self.number_of_Actions, self.actionKnowledge, self.discount,
                               self.rewards) \
               / self.discount

    def update_reward(self, state, reward):
        self.rewards[state] = reward


def value_recursion(state, t, policy, number_of_actions, action_knowledge, discount, rewards):
    """
    :return: discount factor times the value of the state i.e E[ sum_{t=0}^\inf( gamma^t * R_{t+1} ) ]
    """
    probabilities = policy[state]
    total_value = 0
    print t
    for action in range(number_of_actions):
        probability = probabilities[action]
        if probabilities[action] > 0.01 and discount ** t > DISCOUNT_THRESHOLD:
            next_state = tuple(action_knowledge[state + (action,)].tolist())
            next_reward = rewards[next_state]
            total_value += probability * (
                next_reward
                + discount
                * value_recursion(next_state, t + 1, policy, number_of_actions, action_knowledge, discount, rewards)
            )
    return total_value
