from gym.envs.registration import register

register(
    id='MujocoQuadForce-v0',
    entry_point='gym_reinmav.envs.mujoco:MujocoQuadEnv',
)

register(
    id='MujocoQuadForce-v1',
    entry_point='gym_reinmav.envs.mujoco:MujocoQuadHoveringEnv',
)

register(
    id='MujocoQuadForce-v2',
    entry_point='gym_reinmav.envs.mujoco:MujocoQuadHoveringEnv_test',
)

register(
    id='MujocoQuadQuat-v0',
    entry_point='gym_reinmav.envs.mujoco:MujocoQuadQuaternionEnv',
)
