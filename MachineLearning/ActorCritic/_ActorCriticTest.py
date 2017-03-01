import unittest as test
from ActorCritic import ActorCritic
import _Actor
import numpy as np

state1 = (2, 3)
state2 = (3, 4)
state3 = (4, 5)
action = 2
stateAction = state1 + (action,)


class TestSoftmax(test.TestCase):
    def test_softmax(self):
        tderrors = np.asarray([1, 3, 5])
        out = _Actor.softmax(tderrors)
        self.assertEqual(1, np.sum(out))
        self.assertEqual(out.argmax(), 2)


class UnitTests(test.TestCase):
    def __init__(self):
        super(UnitTests, self).__init__()
        self.actions = 2
        self.horizon = 2
        self.stateDims = (3,)
        self.discount = 0.7
        self.buffer = 5
        self.ac = ActorCritic(self.actions, self.horizon, self.stateDims, self.discount, self.buffer)

    def setUp(self):
        self.ac = ActorCritic(self.actions, self.horizon, self.stateDims, self.discount, self.buffer)
        self.ac.critic.update_reward((0,), -1)
        self.ac.critic.update_reward((1,), 0)
        self.ac.critic.update_reward((2,), 1)

if __name__ == '__main__':
    test.main()
