import numpy as np

# Call get next action -> perform that action
# -> update state-action to state (action knowledge), critique policy with new state

beta = 4 # a learning parameter

class ActorCritic:

    def __init__(self, numberOfActions, timeHorizon, rewards, discount):
        self.numberOfActions = numberOfActions
        self.actor = Actor(rewards.shape, numberOfActions)
        self.critic = Critic(timeHorizon, rewards, discount)

    def getNextAction(self, state):
        return getNextAction(state, self.actor.policy)

    def critique(self, previousState, previousAction, state):
        self.actor.updateActionKnowledge(previousState, previousAction, state)

        tDError = self.critic.getTDError(previousState, state, self.actor.policy)
        self.actor.updatePolicy(previousState, previousAction, tDError)

class Actor:
    def __init__(self, stateSpaceDimensions, numberOfActions):
        policyDimensions = stateSpaceDimensions + (numberOfActions,)

        self.policy = np.zeros(policyDimensions)

        actionKnowledgeDims = policyDimensions + (len(stateSpaceDimensions),)
        self.actionToState = np.zeros(actionKnowledgeDims)

    def updateActionKnowledge(self, previousState, previousAction, state):
        actionKnowledgeIndex = previousState + (previousAction,)
        previousKnowledge = self.actionToState[actionKnowledgeIndex]
        newKnowledge = np.asarray(state)

        if(previousKnowledge<0).any(): #If no previous knowledge
            self.actionToState[actionKnowledgeIndex] = newKnowledge
        else:
            self.actionToState[actionKnowledgeIndex] = ((beta-1)*previousKnowledge + newKnowledge)/beta

    def updatePolicy(self, state, action, tDError):
        self.policy[state + (action,)] += tDError


class Critic:
    def __init__(self, timeHorizon, rewards, discount):
        self.rewards = rewards
        self.timeHorizon = timeHorizon
        self.discount = discount

    def getTDError(self, state, nextState, policy, actionKnowledge):
        r1 = self.rewards[nextState]
        v1 = self.value(nextState, policy, actionKnowledge)
        v0 = self.value(state, policy, actionKnowledge)
        return r1 + self.discount*v1 - v0

    def value(self, currentState, policy, actionKnowledge):
        value = 0
        state = currentState
        for t in range(0,self.timeHorizon):
            # Get next state
            action = getNextAction(state, policy)
            expectedState = actionKnowledge[state + (action,)]
            state = tuple([round(stateVar) for stateVar in expectedState])

            reward = self.rewards[state]
            value += self.discount**t + reward

        return value


def getNextAction(state, policy):
    actions = policy[state]
    nextAction = np.argmax(actions)
    return nextAction