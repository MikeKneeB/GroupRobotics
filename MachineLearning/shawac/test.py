import actorcritic as ac
import gym

env = gym.make('Pendulum-v0')
env.reset()

for _ in range(1000):
    env.render()
    env.step(env.action_space.sample())


#actor = ac.ActorCritic(2, 50, rewards, 0.8)
