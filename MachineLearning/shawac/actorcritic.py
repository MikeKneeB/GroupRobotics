import numpy as np
class ActorCritic():

    def __init__(self, stateDimensionsList, numberOfActions, timeHorizon):
        self.numberOfActions = numberOfActions
        self.actor = Actor(stateDimensionsList, numberOfActions)
        self.critic = Critic(stateDimensionsList, timeHorizon)

    def getNextAction(self):
        pass

class Actor():
    def __init__(self, stateDimensions, numberOfActions):
        policyDimensions = stateDimensions + (numberOfActions,)
        self.policy = np.zeros(policyDimensions)


class Critic():
    pass