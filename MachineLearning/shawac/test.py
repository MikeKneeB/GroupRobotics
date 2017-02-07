import actorcritic as ac
import gym, time

env = gym.make('Pendulum-v0')
env.reset()

for _ in range(1000):
    time.sleep(0.01)
    env.render()
    env.step(env.action_space.sample())

numberOfActions = 10
timeHorizon = 50
stateDimensions = (20, 20)
discount = 0.85

actor = ac.ActorCritic(numberOfActions, timeHorizon, stateDimensions, discount)