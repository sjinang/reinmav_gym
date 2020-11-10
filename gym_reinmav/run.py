import gym
import gym_reinmav
import numpy as np
from gym_reinmav.envs.mujoco.play import play


env = gym.make('MujocoQuadForest-v0')
# env.reset()

while True:
    env.render()
    action = env.action_space.sample()
    # print(action)
    obs, rew, done, info = env.step(action)
    if done:
        env.reset()