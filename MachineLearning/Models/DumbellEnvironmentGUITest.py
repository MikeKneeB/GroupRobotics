import DumbellEnvironment as de
import random
import time

env = de.Dumbell()
env.reset(0.7)
for i in range(0,1000):
    time.sleep(0.05)
    #random action
    #env.step(random.uniform(-100, 100))

    env.step(0)

    env.render()