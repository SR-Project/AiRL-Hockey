import numpy as np
import torch
from tqdm import tqdm

import mbrl.util.common
import mbrl.models
import mbrl.planning
import mbrl.util

from gym_air_hockey.envs.air_hockey_env_sb3 import AirHockeyEnvSb3

model_save_path = 'model_based_RL.pth'

env = AirHockeyEnvSb3(render=False)

observation_dim = env.observation_space.shape[0]
action_dim = env.action_space.shape[0]

model = mbrl.models.GaussianMLP(
    in_size=observation_dim + action_dim,  # Input: state + action
    hid_size=256,                          # Hidden layer sizes
    out_size=observation_dim,          # Output: mean and variance for next state
    device='cpu'
)

# Hyperparameters
num_episodes = 5000 
num_steps_per_episode = 1000
training_steps = 500

# Buffer to store experiences
states = []
actions = []
rewards = []
next_states = []
dones = []

# Collect experiences
print("Collecting experience...")
for episode in tqdm(range(num_episodes)):
    state, _ = env.reset()
    for step in range(num_steps_per_episode):
        action = env.action_space.sample()  # policy action
        next_state, reward, terminal, done, _ = env.step(action)
        
        states.append(state)
        actions.append(action)
        rewards.append(reward)
        next_states.append(next_state)
        dones.append(done)
        
        state = next_state
        if done:
            break

states = np.array(states, dtype=np.float32)
actions = np.array(actions, dtype=np.float32)
next_states = np.array(next_states, dtype=np.float32)

import torch.nn as nn
import torch.optim as optim

states_tensor = torch.tensor(states, dtype=torch.float32)
actions_tensor = torch.tensor(actions, dtype=torch.float32)
next_states_tensor = torch.tensor(next_states, dtype=torch.float32)

inputs = torch.cat((states_tensor, actions_tensor), dim=1)

loss_fn = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=1e-3)

# Training the model
print("Training the model...")
for step in tqdm(range(training_steps)):
    # Forward pass
    model.train()
    predicted_next_states = model(inputs)

    mean = predicted_next_states[0]         # First element: mean
    variance = predicted_next_states[1]     # Second element: variance

    loss = loss_fn(mean, next_states_tensor)

    optimizer.zero_grad() 
    loss.backward()      
    optimizer.step()    

    if step % 100 == 0:
        print(f"Step {step}, Loss: {loss.item():.4f}")

print("Training completed.")

torch.save(model.state_dict(), model_save_path)
print(f"Model saved to {model_save_path}")