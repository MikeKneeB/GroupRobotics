import numpy as np
class ActorCritic():

    def __init__(self, stateDimensionsList, numberOfActions, timeHorizon):
        self.numberOfActions = numberOfActions
        self.actor = Actor(stateDimensionsList, numberOfActions)
        self.critic = Critic(stateDimensionsList, timeHorizon)

    def critique(self, state):
        tDError = self.critic.getTDError(state, self.actor.policy)
        self.actor.updatePolicy(state, tDError)

    def getNextAction(self, state):
        actions = self.actor.policy[state]
        nextAction = actions.argmax(actions) ##get the action with the largest TD thing???
        assert 0 <= nextAction < self.numberOfActions
        return nextAction

class Actor():
    def __init__(self, stateDimensions, numberOfActions):
        policyDimensions = stateDimensions + (numberOfActions,)
        self.policy = np.zeros(policyDimensions)

    def updatePolicy(self, state, tDError):
        ##something in here
        pass


class Critic():
    def __init__(self, stateDimensionsList, timeHorizon):
        pass

    def getTDError(self, state, policy):
        return 0

    pass