import numpy as np

# Call get next action -> perform that action -> critique with new sate->

class ActorCritic:

    def __init__(self, numberOfActions, timeHorizon, rewards, discount):
        self.numberOfActions = numberOfActions
        self.actor = Actor(rewards.shape, numberOfActions)
        self.critic = Critic(timeHorizon, rewards, discount)

    def getNextAction(self, state):
        return getNextAction(state, self.actor.policy)

    def critique(self, previousState, previousAction, state):
        tDError = self.critic.getTDError(previousState, state, self.actor.policy)
        self.actor.updatePolicy(previousState, previousAction, tDError)

class Actor:
    def __init__(self, stateSpaceDimensions, numberOfActions):
        policyDimensions = stateSpaceDimensions + (numberOfActions,)
        self.policy = np.zeros(policyDimensions)

    def updatePolicy(self, state, action, tDError):
        self.policy[state + (action,)] += tDError


class Critic:
    def __init__(self, timeHorizon, rewards, discount):
        self.rewards = rewards
        self.timeHorizon = timeHorizon
        self.discount = discount

    def getTDError(self, state, nextState, policy):
        return 0

    def value(self, state, policy):
        value = 0
        for t in range(self.timeHorizon):
            reward = self.rewards[state]
            #calc value

            state = state #what am I doing


def getNextAction(state, policy):
    actions = policy[state]
    nextAction = np.argmax(actions)
    return nextAction