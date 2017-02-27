import unittest as test
from ActorCritic import ActorCritic
import Actor
import numpy as np

state1 = (2, 3)
state2 = (3, 4)
state3 = (4, 5)
action = 2
stateAction = state1 + (action,)


class TestStringMethods(test.TestCase):
    def test_softmax(self):
        tderrors = np.asarray([1, 2, 3])
        out = Actor.softmax(tderrors)
        target = np.asarray([np.exp(0.5), np.exp(1), np.exp(1.5)])
        target /= np.sum(target)
        self.assertTrue(np.array_equal(out, target))
        self.assertEqual(1, np.sum(out))


class UnitTests(test.TestCase):
    def __init__(self):
        super(UnitTests, self).__init__()
        self.actions = 4
        self.horizon = 2
        self.stateDims = (4, 4)
        self.discount = 0.7
        self.buffer = 5
        self.ac = ActorCritic(self.actions, self.horizon, self.stateDims, self.discount, self.buffer)

    def setUp(self):
        self.actions = 4
        self.horizon = 2
        self.stateDims = (4, 4)
        self.discount = 0.7
        self.buffer = 5
        self.ac = ActorCritic(self.actions, self.horizon, self.stateDims, self.discount, self.buffer)


if __name__ == '__main__':
    test.main()
