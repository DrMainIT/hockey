import numpy as np
import torch
from stable_baselines3 import PPO

# Load the trained model
model = PPO.load("/Users/francesco/Downloads/gymnasium_env-Hockey-v0_29/gymnasium_env-Hockey-v0.zip")

# Get policy network
policy = model.policy
layers = policy.mlp_extractor.policy_net

# Extract weights from each layer (Linear layers)
weights = []
for layer in layers:
    if isinstance(layer, torch.nn.Linear):
        weights.append({
            'weight': layer.weight.detach().cpu().numpy(),
            'bias': layer.bias.detach().cpu().numpy()
        })

# Also include the final action_net layer (output layer)
output_layer = policy.action_net
weights.append({
    'weight': output_layer.weight.detach().cpu().numpy(),
    'bias': output_layer.bias.detach().cpu().numpy()
})

# Save all weights
np.save("ppo_policy_weights.npy", weights)
print("✅ Weights exported to ppo_policy_weights.npy")
