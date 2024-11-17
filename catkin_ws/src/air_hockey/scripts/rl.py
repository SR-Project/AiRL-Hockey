import time
from stable_baselines3 import PPO, SAC, TD3
from stable_baselines3.common.env_checker import check_env

model = TD3.load("RLRobot/TD3_checkpoints/TD3_1000000_steps.zip")

def move(puck_x, puck_y, puck_dx, puck_dy, mallet_x, mallet_y, mallet_dx, mallet_dy):
    '''px, py = mallet_y, mallet_x
    vx, vy = mallet_dy, mallet_dx

    puck_px, puck_py = puck_y, puck_x
    puck_vx, puck_vy = puck_dy, puck_dx'''

    obs = [puck_x, puck_y, puck_dx, puck_dy, mallet_x, mallet_y, mallet_dx, mallet_dy]

    action, _states = model.predict(obs)

    return action