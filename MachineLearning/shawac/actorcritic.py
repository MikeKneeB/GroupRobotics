import numpy as np


# Call get next action -> perform that action
# -> update state-action to state (action knowledge), critique policy with new state

class ActorCritic:

    def __init__(self, numberOfActions, timeHorizon, stateDimensions, discount, bufferSize):
        """
        Call getNextAction -> perform that action -> call critique
        """
        self.numberOfActions = numberOfActions
        self.actor = _Actor(stateDimensions, numberOfActions, )
        self.critic = _Critic(stateDimensions, numberOfActions, timeHorizon, discount, bufferSize)

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
        self.critic.updateActionKnowledge(previousState, previousAction, state)

        self.critic.updateReward(state, reward)

        tDError = self.critic.getTDError(previousState, state, self.actor.policy, self.critic.actionKnowledge)
        self.actor.updatePolicy(previousState, previousAction, tDError)

    def avgBufferSize(self):
        return self.critic.bufferIndex.mean()

class _Actor:
    def __init__(self, stateDimensions, numberOfActions):
        policyDimensions = stateDimensions + (numberOfActions,)
        self.policy = np.zeros(policyDimensions)

    def updatePolicy(self, state, action, tDError):
        #print "Policy: ", self.policy[state],
        self.policy[state + (action,)] += tDError
        #print " to ", self.policy[state]


class _Critic:
    def __init__(self, stateDimensions, numberOfActions, timeHorizon, discount, bufferSize):
        self.rewards = np.zeros(stateDimensions)
        self.timeHorizon = timeHorizon
        self.discount = discount

        self.bufferSize = bufferSize
        stateActionDims = stateDimensions + (numberOfActions,)
        actionKnowledgeDims = stateActionDims + (len(stateDimensions),)
        self.actionKnowledge = np.zeros(actionKnowledgeDims, dtype=np.int_)
        actionKnowledgeBufferDims = stateActionDims + (len(stateDimensions)+1, bufferSize)
        self.actionKnowledgeBuffer = np.zeros(actionKnowledgeBufferDims, dtype=np.int_)
        self.bufferIndex = np.zeros(stateActionDims, dtype=np.int_)

    def updateActionKnowledge(self, previousState, previousAction, state):
        stateAction = previousState + (previousAction,)
        newKnowledge = np.asarray(state).T
        numberInBuffer = self.bufferIndex[stateAction]

        print "\n"
        print self.actionKnowledgeBuffer[stateAction]
        print "New knowledge: ", newKnowledge
        observationLocation = -1
        # check for next state in buffer
        for i in range(0, numberInBuffer):
            index = stateAction+(slice(1,3), i)
            if np.array_equal(self.actionKnowledgeBuffer[index], newKnowledge):
                observationLocation = i

        if observationLocation < 0:
            if numberInBuffer < self.bufferSize:
                self.actionKnowledgeBuffer[stateAction + (slice(1, 3), numberInBuffer)] = newKnowledge
                self.actionKnowledgeBuffer[stateAction + (0, numberInBuffer)] += 1
                self.bufferIndex[stateAction] += 1
        else:
            self.actionKnowledgeBuffer[stateAction+(0,observationLocation)] +=1

        modeIndex = np.argmax(self.actionKnowledgeBuffer[stateAction + (0, Ellipsis)])
        modeAction = self.actionKnowledgeBuffer[stateAction + (slice(1,3), modeIndex)]

        print self.actionKnowledgeBuffer[stateAction]
        print "Mode state after action: ", modeAction
        print "\n"

        self.actionKnowledge[stateAction] = modeAction
        #print "Next state: ", self.actionKnowledge[stateAction]


    def getTDError(self, previousState, newState, policy, actionKnowledge):
        r1 = self.rewards[newState]
        #print "Reward of new state: ", r1
        v1 = self.value(newState, policy, actionKnowledge)
        #print "Value of new state: ", v1
        v0 = self.value(previousState, policy, actionKnowledge)
        #print "Value of previous state: ", v0
        return r1 + self.discount*v1 - v0

    def value(self, state, policy, actionKnowledge):
        value = 0
        for t in range(1,self.timeHorizon+1):
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

def wrapAroundAverage(numpyIn, wrap):
    pass