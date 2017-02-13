import numpy as np

# Call get next action -> perform that action
# -> update state-action to state (action knowledge), critique policy with new state

class ActorCritic:

    def __init__(self, numberOfActions, timeHorizon, stateDimensions, discount, bufferSize):
        """
        Call getNextAction -> perform that action -> call critique
        """
        self.numberOfActions = numberOfActions
        self.actor = _Actor(stateDimensions, numberOfActions, bufferSize)
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
        #print "state: ", state
        self.actor.updateActionKnowledge(previousState, previousAction, state)

        self.critic.updateReward(state, reward)

        tDError = self.critic.getTDError(previousState, state, self.actor.policy, self.actor.actionKnowledge)
        self.actor.updatePolicy(previousState, previousAction, tDError)

    def avgBufferSize(self):
        return self.actor.bufferIndex.mean()

class _Actor:
    def __init__(self, stateSpaceDimensions, numberOfActions, bufferSize):
        policyDimensions = stateSpaceDimensions + (numberOfActions,)
        self.policy = np.zeros(policyDimensions)
        self.bufferSize = bufferSize

        actionKnowledgeDims = policyDimensions + (len(stateSpaceDimensions),)
        self.actionKnowledge = np.zeros(actionKnowledgeDims, dtype=np.int_)
        actionKnowledgeBufferDims = actionKnowledgeDims + (bufferSize,)
        self.actionKnowledgeBuffer = -np.ones(actionKnowledgeBufferDims, dtype=np.int_)
        self.bufferIndex = np.zeros(policyDimensions, dtype=np.int_)

    def updateActionKnowledge(self, previousState, previousAction, state):
        stateAction = previousState + (previousAction,)

        if self.bufferIndex[stateAction] < self.bufferSize:
            bufferTop = stateAction + (Ellipsis, self.bufferIndex[stateAction])
            newKnowledge = np.asarray(state)
            self.actionKnowledgeBuffer[bufferTop] = newKnowledge

            #print self.actionKnowledgeBuffer[stateAction]

            filledBufferIndexes = stateAction + (Ellipsis, slice(0,self.bufferIndex[stateAction]+1))
            filledBuffer = self.actionKnowledgeBuffer[filledBufferIndexes]

            # print "state action: ", stateAction
            # print "action knowledge: ", self.actionKnowledge[stateAction]
            #print "Filled buffer: ", self.actionKnowledgeBuffer[filledBufferIndexes]
            stateFromAction = np.mean(filledBuffer, axis=1)

            self.actionKnowledge[stateAction] = stateFromAction.round().astype(int)


            self.bufferIndex[stateAction] += 1


    def updatePolicy(self, state, action, tDError):
        #print "Policy: ", self.policy[state],
        self.policy[state + (action,)] += tDError
        #print " to ", self.policy[state]


class _Critic:
    def __init__(self, timeHorizon, stateDimensions, discount):
        self.rewards = np.zeros(stateDimensions)
        self.timeHorizon = timeHorizon
        self.discount = discount

    def getTDError(self, previousState, newState, policy, actionKnowledge):
        r1 = self.rewards[newState]
        #print "Reward of new state: ", r1
        v1 = self.value(newState, policy, actionKnowledge)
        #print "Value of new state: ", v1
        v0 = self.value(previousState, policy, actionKnowledge)
        #print "Value of previous state: ", v0
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
            value += self.discount**t * reward

        return value

    def updateReward(self, state, reward):
        #print "Updated state ", state, " reward from ", self.rewards[state], " to ", reward
        #print "Current reward space: ", self.rewards
        self.rewards[state] = reward


def getNextAction(state, policy):
    actions = policy[state]
    nextAction = np.argmax(actions)
    return nextAction
