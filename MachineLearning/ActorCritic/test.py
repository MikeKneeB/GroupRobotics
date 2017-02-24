import ActorCritic as shawac
import numpy as np
import unittest

state1 = (2,3)
state2 = (3,4)
state3 = (4,5)
action = 2
stateAction = state1 + (action,)

class UnitTests(unittest.TestCase):
    def setUp(self):
        self.actions = 4
        self.horizon = 2
        self.stateDims = (4, 4)
        self.discount = 0.7
        self.buffer = 5
        self.ac = shawac.ActorCritic(self.actions, self.horizon, self.stateDims, self.discount, self.buffer)

if __name__ == '__main__':
    unittest.main()