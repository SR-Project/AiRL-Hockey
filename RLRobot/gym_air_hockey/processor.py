from PIL import Image
import numpy as np

from air_hockey_game.src.dimensions import D
import air_hockey_game.src.phy_const as P


def normalize_observations(puck_pos, puck_vel, mallet_pos, mallet_vel):
    # Normalize puck position (x, y)
    puck_x_norm = (2 * puck_pos[0] / D.width) - 1
    puck_y_norm = (2 * puck_pos[1] / D.height) - 1

    # Normalize mallet position (x, y)
    mallet_x_norm = (2 * mallet_pos[0] / D.width) - 1
    mallet_y_norm = (2 * mallet_pos[1] / D.height) - 1

    # Normalize puck velocity (dx, dy)
    puck_dx_norm = puck_vel[0] / P.puck_maximum_speed
    puck_dy_norm = puck_vel[1] / P.puck_maximum_speed

    # Normalize mallet velocity (dx, dy)
    mallet_dx_norm = mallet_vel[0] / P.mallet_maximum_speed
    mallet_dy_norm = mallet_vel[1] / P.mallet_maximum_speed

    # Return all normalized values as a flat array
    return np.array([puck_x_norm, puck_y_norm]), np.array([puck_dx_norm, puck_dy_norm]), np.array([mallet_x_norm, mallet_y_norm]), np.array([mallet_dx_norm, mallet_dy_norm])


class DataProcessor(object):
    def __init__(self):
        # [-1, -1] = go left and down, [-1, 0] go left and stay, [1, -1] go right and down ...
        self.actions = np.array([
                            [-1, -1],
                            [-1,  0],
                            [-1,  1],
                            [ 0, -1],
                            [ 0,  0],
                            [ 0,  1],
                            [ 1, -1],
                            [ 1,  0],
                            [ 1,  1],
                            [ 0,  0]],
                            dtype=np.int8)
        self.metrics = []
        self.metrics_names = []

    def process_step(self, observation, reward, done, info):
        observation = self.process_observation(observation)
        reward = self.process_reward(reward)
        info = self.process_info(info)
        return observation, reward, done, info

    def process_observation(self, puck_info, mallet_info):
        puck_position, puck_velocity = puck_info
        mallet_position, mallet_velocity = mallet_info

        # normalization
        puck_position, puck_velocity, mallet_position, mallet_velocity = normalize_observations(puck_position, puck_velocity, mallet_position, mallet_velocity)

        state = np.concatenate([
            puck_position,
            puck_velocity,
            mallet_position,
            mallet_velocity
        ]).astype(np.float32)
        return state

    def process_reward(self, reward):
        return np.clip(reward, -1.0, 1.0)

    def process_info(self, info):
        return info

    # def process_action(self, label):
    #     if label is None: return None
    #     return self.actions[label]
    def process_action(self, action):
        if action is None:
            return None # AI plays
        else:
            # Assuming actions are in the range [-1, 1], scale them appropriately
            # max_speed = 6
            # return np.clip(action * max_speed, -self.max_speed, self.max_speed)
            return action

    def process_state_batch(self, batch):
        _, depth, height, width = batch[0].shape
        batch = np.array(batch).reshape(len(batch), depth, height, width)
        return batch

    # Used to generate labels for the neural network
    def action_to_label(self, action):
        return int(action[0]*3 + action[1] + 4)