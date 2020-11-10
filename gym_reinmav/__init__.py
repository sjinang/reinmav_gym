from gym.envs.registration import register

register(
    id='MujocoQuadReach-v0',
    entry_point='gym_reinmav.envs.mujoco:MujocoQuadReachEnv',
)


register(
    id='MujocoQuadForest-v0',
    entry_point='gym_reinmav.envs.mujoco:MujocoQuadForestEnv',
)

