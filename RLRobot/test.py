import time
from stable_baselines3 import PPO, SAC, TD3
from stable_baselines3.common.env_checker import check_env

from gym_air_hockey.envs.air_hockey_env_sb3 import AirHockeyEnvSb3

# Load the PPO model
# model = TD3.load("TD3_air_hockey")
model = TD3.load("RLRobot/TD3_checkpoints/TD3_300000_steps.zip")

# Initialize environment with rendering enabled
env = AirHockeyEnvSb3(render=True)

obs, info = env.reset()
print("RESET________________-")
for i in range(10000):
    action, _states = model.predict(obs)
    obs, rewards, done, _, info = env.step(action)

    if done:
        obs, info = env.reset()

env.close()
