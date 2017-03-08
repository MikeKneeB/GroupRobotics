from _Actor import Actor
from _Critic import Critic


class ActorCritic:
    def __init__(self, number_of_actions, state_dimensions, discount, learning_rate, temperature_parameter):
        """
        Call getNextAction -> perform that action -> call critique
        """
        self.numberOfActions = number_of_actions
        self.actor = Actor(number_of_actions, state_dimensions, temperature_parameter)
        self.critic = Critic(state_dimensions, discount=discount, value_learning_rate=learning_rate)

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

        td_error = self.critic.get_td_error(previous_state, state, reward)
        self.actor.update_policy(previous_state, previous_action, td_error)
        self.critic.update_value(previous_state, td_error)

