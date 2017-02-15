import actorcritic as shawac
import numpy as np
import unittest

state1 = (2,3)
state2 = (3,4)
state3 = (4,5)
action = 2
stateAction = state1 + (action,)

class Test(unittest.TestCase):
    def setUp(self):
        self.actions = 4
        self.horizon = 2
        self.stateDims = (4, 4)
        self.discount = 0.7
        self.buffer = 5
        self.ac = shawac.ActorCritic(self.actions, self.horizon, self.stateDims, self.discount, self.buffer)

    def testKnowledgeUpdated(self):
        actor = self.ac.actor
        actor.updateActionKnowledge(state1, action, state2)

        self.assertEqual(actor.bufferIndex[stateAction], 1)

        bufferHead = actor.actionKnowledgeBuffer[stateAction + (Ellipsis, actor.bufferIndex[stateAction]-1)]
        self.assertTrue(np.array_equal(bufferHead, np.asarray(state2)))
        self.assertTrue(np.array_equal(actor.actionKnowledge[stateAction], np.asarray(state2)))

    def testValue(self):
        critic = self.ac.critic
        actor = self.ac.actor
        reward = 10

        critic.updateReward((0,0), reward)

        expectedValue = self.discount*reward + self.discount**2*reward
        self.assertEqual(critic.value(state1, actor.policy, actor.actionKnowledge), expectedValue)

if __name__ == '__main__':
    unittest.main()