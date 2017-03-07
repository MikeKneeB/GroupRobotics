import unittest as test
import _Actor
import numpy as np


class TestSoftmax(test.TestCase):
    def test_softmax(self):
        tderrors = np.asarray([1, 3, 5])
        out = _Actor.softmax(tderrors, 1)
        self.assertEqual(1, np.sum(out))
        self.assertEqual(out.argmax(), 2)

if __name__ == '__main__':
    test.main()
