from _Actor import Actor
from _Critic import Critic


class ActorCritic:
    def __init__(self, numberOfActions, timeHorizon, stateDimensions, discount, bufferSize):
        """
        Call getNextAction -> perform that action -> call critique
        """
        self.numberOfActions = numberOfActions
        self.actor = Actor(stateDimensions, numberOfActions, )
        self.critic = Critic(stateDimensions, numberOfActions, timeHorizon, discount, bufferSize)

    def get_next_action(self, state):
        """
        :param state: tuple of integers, from 0 to the state dimension size
        """
        return self.actor.get_next_action(state)

    def critique(self, previous_state, previous_action, state, reward):
        """
        :param previous_state: tuple of integers, from 0 to the state dimension size
        :param previous_action: an integer from 0 to numberOfActions
        :param state: tuple of integers, from 0 to the state dimension size
        :param reward: arbitrary double, reward of this state
        """
        # print "state: ", state
        self.critic.update_action_knowledge(previous_state, previous_action, state)

        self.critic.update_reward(state, reward)

        td_error = self.critic.get_td_error(previous_state, state, self.actor.policy, self.critic.actionKnowledge)
        self.actor.update_policy(previous_state, previous_action, td_error)

    def avg_buffer_size(self):
        return self.critic.bufferIndex.mean()


