# **********************************************************************
#
# Copyright (c) 2019, Autonomous Systems Lab
# Author: Dongho Kang <eastsky.kang@gmail.com>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# *************************************************************************
import numpy as np
import os
from gym import utils
# from gym.envs.mujoco import mujoco_env
from gym import spaces
from gym_reinmav.envs.mujoco.mujoco_goal_env import Mujoco_Goal_Env


class MujocoQuadReachEnv(Mujoco_Goal_Env, utils.EzPickle):
    def __init__(self, xml_name="quadrotor_env.xml",
                reward_type='sparse',max_episode_steps=50,
                range_min=1,range_max=2,threshold=0.5):

        self.range_min = range_min
        self.range_max = range_max
        self.reward_type = reward_type
        self.threshold = threshold
        self._max_episode_steps = max_episode_steps # giving error for not using _ before
        
        self.seed()
        self.goal = self._sample_goal() 
        
        xml_path = os.path.join(os.path.dirname(__file__), "./assets", xml_name)

        utils.EzPickle.__init__(self)
        Mujoco_Goal_Env.__init__(self, xml_path, 9)

        self.sim.model.site_pos[self.sim.model.site_name2id("goal_site")] = self.goal
        ### enforcing initial state
        # self.set_state(np.array([0.0,0.0,2.0,1.0,0.0,0.0,0.0]),np.array([0.0]*6))


    def _step(self, a):
        self.do_simulation(self.clip_action(a), self.frame_skip)
        return self.sim.data.ncon
    
    def step(self, a):
        assert ((a[0]+1)/2)>-0.00001
        
        dv = 0.15*((a[0]+1)/2) #Only forward velocity HER due to max_u parameter in config of HER, don't use this with action_sample
        # dv = 0.15*a[0]
        dw = 0.25*a[1]
        c = 0.73575
        
        prev_ncon = self.sim.data.ncon
        con = []

        con.append(self._step([c-dv, c+dv, c+dv, c-dv]))
        con.append(self._step([c+dv, c-dv, c-dv, c+dv]))
        con.append(self._step([c+dv, c-dv, c-dv, c+dv]))
        con.append(self._step([c-dv, c+dv, c+dv, c-dv]))
        pos = self.sim.data.qpos
        vel = self.sim.data.qvel
        self.set_state(np.array([pos[0],pos[1],2,pos[3],0,0,pos[6]]),np.array([0.0]*6))
        # self.render()

        con.append(self._step([c+dw, c-dw, c+dw, c-dw]))
        con.append(self._step([c-dw, c+dw, c-dw, c+dw]))
        pos = self.sim.data.qpos
        vel = self.sim.data.qvel
        self.set_state(np.array([pos[0],pos[1],2,pos[3],0,0,pos[6]]),np.array([0.0]*6))
        
        ob = self._get_obs()
        
        # print(self.sim.data.sensordata)

        reward = self.compute_reward(ob['achieved_goal'],ob['desired_goal'],{})
        dist = np.linalg.norm(ob['achieved_goal'] - ob['desired_goal'], axis=-1)

        notdone = np.isfinite(ob['observation']).all() \
                  and abs(ob['observation'][0]) < 5.0 \
                  and abs(ob['observation'][1]) < 5.0 \
                  and abs(dist) > self.threshold 
        
        con.append(self.sim.data.ncon)
        collision = max(con) > prev_ncon

        done = (not notdone) or collision

        info = {
            'is_success': (abs(dist) < self.threshold).astype(np.float32),
        }

        # for _ in range(50):
        #     self.render()
        
        return ob, reward, done, info

    def compute_reward(self, achieved_goal, desired_goal,info=None):
        # Compute distance between goal and the achieved goal.
        assert achieved_goal.shape == desired_goal.shape
        d = np.linalg.norm(achieved_goal - desired_goal, axis=-1)

        if self.reward_type == 'sparse':
            return -(d > self.threshold).astype(np.float32)
        else:
            return -d

    def clip_action(self, action):
        """
        clip action to [0, inf]
        :param action:
        :return: clipped action
        """
        action = np.clip(action, a_min=0, a_max=np.inf)
        return action

    def reset_model(self):
        # script.run()
        # self.__init__()
        
        qpos = self.init_qpos
        qvel = self.init_qvel
        self.set_state(qpos, qvel)

        self.goal = self._sample_goal()
        self.sim.model.site_pos[self.sim.model.site_name2id("goal_site")] = self.goal

        ### enforcing intial conditions
        # self.set_state(np.array([0.0,0.0,2.0,1.0,0.0,0.0,0.0]),np.array([0.0]*6))
        
        return self._get_obs()

    def reset(self):
        self.sim.reset()
        ob = self.reset_model()
        # print('*',ob)
        return ob

    def _get_obs(self):
        obs = np.concatenate([self.sim.data.qpos, self.sim.data.qvel]).ravel()
        return {
            'observation': obs.copy(),
            'achieved_goal': obs[:3].copy(),
            'desired_goal': self.goal.copy(),
        }
    
    def _sample_goal(self):
        goal = self.np_random.uniform(-self.range_max, self.range_max, size=3)
        while np.sqrt((goal[0]**2)+(goal[1]**2))<self.range_min:
            goal = self.np_random.uniform(-self.range_max, self.range_max, size=3)
        # goal = np.array([0,4,0])
        goal[2] = 2
        # Forward Goal
        # goal[1] = abs(goal[1]) 
        return goal.copy()

    # def viewer_setup(self):
    #     v = self.viewer
    #     v.cam.trackbodyid = 0
    #     v.cam.distance = self.model.stat.extent * 10

    def viewer_setup(self):
        self.viewer.trackbodyid=0
        self.viewer.cam.distance = self.model.stat.extent * 2

    @property
    def mass(self):
        return self.model.body_mass[1]

    @property
    def gravity(self):
        return self.model.opt.gravity
