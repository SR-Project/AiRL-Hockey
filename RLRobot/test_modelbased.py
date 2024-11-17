import numpy as np
import torch
import mbrl.models
from gym_air_hockey.envs.air_hockey_env_sb3 import AirHockeyEnvSb3

model_save_path = 'model_based_RL.pth'  # Path where the model was saved

env = AirHockeyEnvSb3(render=True)

observation_dim = env.observation_space.shape[0]
action_dim = env.action_space.shape[0]

model = mbrl.models.GaussianMLP(
    in_size=observation_dim + action_dim,
    hid_size=256,
    out_size=observation_dim,
    device='cpu'
)

# Load the model state
model.load_state_dict(torch.load(model_save_path))
model.eval()

# Run predictions
state, _ = env.reset()
done = False

while not done:
    action = env.action_space.sample() # here should be done with predicted action

    input_tensor = torch.tensor(np.concatenate((state, action)), dtype=torch.float32).unsqueeze(0)

    with torch.no_grad():
        predicted_next_state = model(input_tensor)
        mean = predicted_next_state[0].numpy() 

    next_state, reward, terminal, done, _ = env.step(action) 
    state = next_state 

env.close()
print("Test completed.")
