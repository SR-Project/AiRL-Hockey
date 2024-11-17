import os
import numpy as np

from stable_baselines3 import PPO, SAC, TD3
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.callbacks import CheckpointCallback

from gym_air_hockey.envs.air_hockey_env_sb3 import AirHockeyEnvSb3

# Initialize environment with rendering disabled
env = AirHockeyEnvSb3(render=True)

check_env(env, warn=False)

# model = SAC("MlpPolicy", 
#             env, 
#             verbose=1, 
#             learning_rate=3e-5,
#             action_noise=None)
# model = PPO(
#     "MlpPolicy",
#     env,
#     verbose=1,
#     learning_rate=1e-4,
#     n_steps=4096,
#     batch_size=2048,
#     gamma=0.90,
#     gae_lambda=0.95,
#     clip_range=0.2,
#     ent_coef=0.0,
#     tensorboard_log="RLRobot/tensorboard"
# )

n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

model = TD3(
    "MlpPolicy",
    env,
    learning_rate=1e-4,
    batch_size=256,
    tau=0.001,
    buffer_size=int(1e7),
    policy_delay=3,
    action_noise=action_noise,
    # gradient_steps=100,
    verbose=1,
    tensorboard_log="RLRobot/tensorboard"
)

checkpoint_callback = CheckpointCallback(
  save_freq=100000,
  save_path="./RLRobot/TD3_checkpoints",
  name_prefix="TD3",
  save_replay_buffer=True,
  save_vecnormalize=True,
)

# Train the model
model.learn(total_timesteps=1000000, callback=checkpoint_callback)

# Save the trained model
model.save("TD3_air_hockey")

env.close()