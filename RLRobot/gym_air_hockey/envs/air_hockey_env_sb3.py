import gymnasium as gym
import numpy as np
import pygame

from air_hockey_game.src.air_hockey import AirHockey
from gym_air_hockey.processor import DataProcessor
import air_hockey_game.src.phy_const as P
from air_hockey_game.src.dimensions import D


def normalize_positions(pos):
    puck_x_norm = (2 * pos[0] / D.width) - 1
    puck_y_norm = (2 * pos[1] / D.height) - 1

    return np.array([puck_x_norm, puck_y_norm])


class AirHockeyEnvSb3(gym.Env):
    def __init__(self, render=False):
        super(AirHockeyEnvSb3, self).__init__()

        self.game = AirHockey(render=render)
        self.processor = DataProcessor()

        # Continuous action space: dx and dy (movement in X and Y axes)
        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,), # dx and dy
            dtype=np.float32
        )
        # self.action_space = gym.spaces.Discrete(len(self.processor.actions))

        # Observation space: positions and velocities of the puck and mallet
        self.observation_space = gym.spaces.Box(
            # low=np.array([0.0, 0.0, -P.puck_maximum_speed, -P.puck_maximum_speed, 0.0, 0.0, -P.mallet_maximum_speed, -P.mallet_maximum_speed]),
            # high=np.array([D.width, D.height, P.puck_maximum_speed, P.puck_maximum_speed, D.width, D.height, P.mallet_maximum_speed, P.mallet_maximum_speed]),
            low=np.array([-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]),
            high=np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
            dtype=np.float32
        )

        # Internal game state
        self.viewer = None
        self.use_object = {'puck': True, 'top_mallet': True}
        self.reset()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed, options=options)

        # Reset the game environment and retrieve initial state
        game_info = self.game.reset(use_object=self.use_object)
        state = self.processor.process_observation(game_info.puck_info, game_info.mallet_info)

        return state, {}

    def step(self, action, debug=False):
        # Apply the new velocity as the action
        robot_action = self.processor.process_action(action)
        human_action = self.processor.process_action(None)

        # Step the game environment (update the game state)
        game_info = self.game.step(robot_action=robot_action, human_action=human_action, use_object=self.use_object)
        terminal = game_info.scored is not None

        # Reward calculations
        reward = 0.0


        # # Reward for hitting the puck
        # if game_info.puck_was_hit:
        #     if self.game.puck._velocity[1] < 0:  # Moving towards opponent's goal
        #         reward += 2.0
        #     else:  # Moving towards its own goal
        #         reward -= 2.0

        # # Penalty for hitting the border
        # if game_info.hit_the_border:
        #     print("hit the border")
        #     reward -= 5.0

        # # Encourage agent to move towards the puck
        # if game_info.puck_is_at_the_bottom:
        #     distance_to_puck = np.linalg.norm(self.game.puck.position - self.game.bottom_mallet.position)
        #     # print(f"Distance to puck: {distance_to_puck}")
        #     reward -= distance_to_puck * 0.01  # Penalty for being far from the puck
            
        #     # if game_info.distance_decreased:
        #     #     reward += 2.0
        #     # else:
        #     #     reward -= 2.0

        # # Defense process
        # else:
        #     if game_info.in_the_target: # defensive position
        #         if np.sum(robot_action) == 0:
        #             reward = 5
        #         else:
        #             reward = -2
        #     else:
        #         reward = -1

        # Goal rewards
        if game_info.scored == 'top':
            reward -= 5.0  # Penalty for opponent scoring
        elif game_info.scored == 'bottom':
            reward += 5.0  # Reward for scoring a goal

        # Reward for moving toward the puck
        if game_info.puck_is_at_the_bottom:
            distance_to_puck = np.linalg.norm(self.game.puck.position - self.game.bottom_mallet.position)
            reward += max(0, 1 - (distance_to_puck / 10.0))  # Reward approaching puck (scaled)

        # Reward for puck velocity toward the opponent's goal
        puck_velocity = self.game.puck._velocity
        if puck_velocity[1] > 0:  # Positive y-direction toward opponent
            reward += 0.5 * np.clip(puck_velocity[1], 0, 5)  # Reward based on speed toward goal

        # Penalty for inaction
        if np.sum(robot_action) == 0:
            reward -= 0.2  # Discourage standing still

        # Reward for hitting the puck and aiming toward the opponent’s goal
        if game_info.puck_was_hit:
            reward += 1.0  # Increased reward for offensive movement

        # Penalty for hitting the border
        if game_info.hit_the_border:
            # print("hit the border")
            reward -= 0.5

        # Encourage defensive positioning
        if not game_info.puck_is_at_the_bottom:
            if game_info.in_the_target: reward += 1.0  
            else: reward = -0.5
    
        reward = np.clip(reward, -10.0, 10.0)
        
        # print(f"Final Reward: {reward}")

        # Process the next state
        state = self.processor.process_observation(game_info.puck_info, game_info.mallet_info)

        return state, reward, terminal, False, {
            'robot_action': robot_action,
            'human_action': human_action
        }

    def render(self, mode='human'):
        if self.game.render:
            pass

    def close(self):
        if self.game.render:
            pygame.quit()
