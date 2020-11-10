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
from gym.envs.mujoco import mujoco_env
from gym import spaces
from gym_reinmav.envs.mujoco.mujoco_goal_env import Mujoco_Goal_Env
from gym_reinmav.envs.mujoco import script


class MujocoQuadReachEnv(Mujoco_Goal_Env, utils.EzPickle):
    def __init__(self, xml_name="quadrotor_env_modified.xml",version=0,range=10,reward_type='sparse',threshold=0.5):

        self.range = range
        self.reward_type = reward_type
        self.threshold = threshold
        
        self.seed()

        self.goal = self._sample_goal()
        script.run(version,self.goal,self.threshold)
        
        xml_path = os.path.join(os.path.dirname(__file__), "./assets", xml_name)

        utils.EzPickle.__init__(self)
        Mujoco_Goal_Env.__init__(self, xml_path, 6)

    def _step(self, a):
        self.do_simulation(self.clip_action(a), self.frame_skip)
        return self.sim.data.ncon
    
    def step(self, a):
        dv = 0.15*a[0]
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
        self.render()

        con.append(self._step([c+dw, c-dw, c+dw, c-dw]))
        con.append(self._step([c-dw, c+dw, c-dw, c+dw]))
        pos = self.sim.data.qpos
        vel = self.sim.data.qvel
        self.set_state(np.array([pos[0],pos[1],2,pos[3],0,0,pos[6]]),np.array([0.0]*6))

        ob = self._get_obs()
        
        # print(self.sim.data.sensordata)

        reward = self.compute_reward(ob['achieved_goal'],ob['desired_goal'])

        notdone = np.isfinite(ob['observation']).all() \
                  and abs(ob['observation'][0]) < 10.0 \
                  and abs(ob['observation'][1]) < 10.0 \
                  and abs(reward) > self.threshold 

        con.append(self.sim.data.ncon)
        collision = max(con) > prev_ncon

        done = (not notdone) or collision
        return ob, reward, done, {}

    def compute_reward(self, achieved_goal, desired_goal):
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

    # def reset_model(self):
    #     qpos = self.init_qpos
    #     qvel = self.init_qvel
    #     self.set_state(qpos, qvel)
    #     return self._get_obs()

    def reset_model(self):
        # script.run()
        self.__init__()
        
        qpos = self.init_qpos
        qvel = self.init_qvel
        self.set_state(qpos, qvel)
        
        return self._get_obs()

    def reset(self):
        self.sim.reset()
        ob = self.reset_model()
        # print('*',ob)
        return ob
    
    def _sample_goal(self):
        goal = self.np_random.uniform(-self.range, self.range, size=3)
        goal[2] = 2
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
