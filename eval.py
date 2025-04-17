import gymnasium as gym
import gymnasium_env
from stable_baselines3 import PPO
from icecream import ic 
"""
env = gym.make("gymnasium_env/Hockey-v0", render_mode="rgb_array")

model = PPO.load("/Users/francesco/Desktop/hockey/logs/ppo/gymnasium_env-Hockey-v0_7/gymnasium_env-Hockey-v0.zip",env=env)   

vec_env = model.get_env()
obs = vec_env.reset()
for i in range(1000):
    action, _state = model.predict(obs, deterministic=False)
    obs, reward, done, info = vec_env.step(action)
    vec_env.render("human")
    if done:
      obs = vec_env.reset()
"""

env = gym.make("gymnasium_env/Hockey-v0", render_mode="human")

env.reset()

for _ in range(1000):
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    env.render()
    if terminated or truncated:
        obs = env.reset()
env.close()