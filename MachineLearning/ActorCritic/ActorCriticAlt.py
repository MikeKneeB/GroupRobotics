import tensorflow as tf

from MachineLearning import DumbellEnvironment as dumenv
from MachineLearning.ActorCritic import actor, critic, ActorCriticTrain as ACT

def main():
    with tf.Session() as sess:
        env = dumenv.Dumbell(length=1.0, mass=1.0)

        state_dim = 2
        action_dim = 1
        max_action = 100


        # Build our actor and critic agents.
        actor_model = actor.ActorNetwork(sess, state_dim, action_dim, max_action, 0.0001, 0.001)
        critic_model = critic.CriticNetwork(sess, state_dim, action_dim, max_action, 0.001, 0.001, actor_model.get_num_trainable_vars())

        # Train.
        ACT.train(sess, actor_model, critic_model, env, state_dim, action_dim, max_action, epochs=50, run_length=300, render=False)

if __name__ == '__main__':
    main()
