import numpy as np
class ActorCritic():

    def __init__(self, stateDimensionsList, numberOfActions, timeHorizon):
        self.numberOfActions = numberOfActions
        self.actor = Actor(stateDimensionsList, numberOfActions)
        self.critic = Critic(stateDimensionsList, timeHorizon)



    def getNextAction(self, state):
        error = self.critic.getTDError(state, self.actor.policy)


class Actor():
    def __init__(self, stateDimensions, numberOfActions):
        policyDimensions = stateDimensions + (numberOfActions,)
        self.policy = np.zeros(policyDimensions)

    def updatePolicy(self):
        pass

class Critic():
    def getTDError(self, state, policy):
        return 0

    pass