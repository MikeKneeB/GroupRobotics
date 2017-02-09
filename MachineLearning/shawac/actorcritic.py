import numpy as np

# Call get next action -> perform that action
# -> update state-action to state (action knowledge), critique policy with new state

class ActorCritic:

    def __init__(self, numberOfActions, timeHorizon, stateDimensions, discount):
        """
        Call getNextAction -> perform that action -> call critique
        """
        self.numberOfActions = numberOfActions
        self.actor = _Actor(stateDimensions, numberOfActions)
        self.critic = _Critic(timeHorizon, stateDimensions, discount)

    def getNextAction(self, state):
        """
        :param state: tuple of integers, from 0 to the state dimension size
        """
        return getNextAction(state, self.actor.policy)

    def critique(self, previousState, previousAction, state, reward):
        """
        :param previousState: tuple of integers, from 0 to the state dimension size
        :param previousAction: an integer from 0 to numberOfActions
        :param state: tuple of integers, from 0 to the state dimension size
        :param reward: arbitrary double
        """
        self.actor.updateActionKnowledge(previousState, previousAction, state)

        self.critic.updateReward(state, reward)

        tDError = self.critic.getTDError(previousState, state, self.actor.policy, self.actor.actionToState)
        self.actor.updatePolicy(previousState, previousAction, tDError)

class _Actor:
    def __init__(self, stateSpaceDimensions, numberOfActions):
        policyDimensions = stateSpaceDimensions + (numberOfActions,)
        self.policy = np.zeros(policyDimensions)

        actionKnowledgeDims = policyDimensions + (len(stateSpaceDimensions),)
        self.actionToState = -np.ones(actionKnowledgeDims)

    def updateActionKnowledge(self, previousState, previousAction, state):
        actionKnowledgeIndex = previousState + (previousAction,)
        newKnowledge = np.asarray(state)
        #print "Updated state-action ", actionKnowledgeIndex, " state from ", self.actionToState[actionKnowledgeIndex], " to ", newKnowledge
        self.actionToState[actionKnowledgeIndex] = newKnowledge

    def updatePolicy(self, state, action, tDError):
        #print "Policy: ", self.policy[state],
        self.policy[state + (action,)] += tDError
        #print " to ", self.policy[state]


class _Critic:
    def __init__(self, timeHorizon, stateDimensions, discount):
        self.rewards = np.zeros(stateDimensions)
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
            state = tuple([int(round(stateVar)) for stateVar in expectedState])

            reward = self.rewards[state]
            value += self.discount**t + reward

        return value

    def updateReward(self, state, reward):
        #print "Updated state ", state, " reward from ", self.rewards[state], " to ", reward
        self.rewards[state] = reward


def getNextAction(state, policy):
    actions = policy[state]
    nextAction = np.argmax(actions)
    return nextAction