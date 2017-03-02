"""
Author: Chris Patmore
Date: 02/03/2017
Description: Replay Memory implementation
"""
from collections import deque
import random

# A simple class to store experiences
# an experience is a state, action, reward and end-state


class ReplayMemory(object):

    # constructor using given parameters
    def __init__(self, buffer_size, random_seed=111):
        self.buffer_size = buffer_size
        self.count = 0
        self.buffer = deque()
        if random_seed != 0:
            random.seed(random_seed)

    # add an experience to memory
    def add(self, init_state, action, reward, final_state):
        experience = (init_state, action, reward, final_state)

        # if buffer limit not reached append
        if self.count < self.buffer_size:
            self.buffer.append(experience)
            self.count += 1

        # if buffer limit has been reached pop one off the front and append to the back
        else:
            self.buffer.popleft()
            self.buffer.append(experience)

    # size of the current memory
    def size(self):
        return self.count

    # return a random sample of the experiences in memory
    def sample(self, batch_size):

        if self.count < batch_size:
            batch = random.sample(self.buffer, self.count)

        else:
            batch = random.sample(self.buffer, batch_size)

        start_state_batch = [_[0] for _ in batch]
        action_batch = [_[1] for _ in batch]
        reward_batch = [_[2] for _ in batch]
        final_state_batch = [_[3] for _ in batch]

        return start_state_batch, action_batch, reward_batch, final_state_batch
