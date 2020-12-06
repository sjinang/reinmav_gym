import gym
import gym_reinmav
import numpy as np
from gym_reinmav.envs.mujoco.play import play


env = gym.make('MujocoQuadReach-v0')
orig = env.sim.data.qpos

for i in range(50):
    for j in range(20):
        env.render()
    # action = env.action_space.sample()
    action = np.array([0.5,1])
    # if i>12:
    #     action = np.array([1,0])
    obs, rew, done, info = env.step(action)
    print(i,obs["observation"][1])
    if i==20:
        env.reset()
    
    # if(abs(obs["observation"][6])>0.5):
    #     break
    # print(action)
    # print(i,obs['desired_goal'])
    # if done:
    #     env.reset()

while True:
    env.render()