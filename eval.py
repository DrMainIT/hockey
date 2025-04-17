import gymnasium as gym
import gymnasium_env
from stable_baselines3 import PPO
from icecream import ic 

env = gym.make("gymnasium_env/Hockey-v0", render_mode="rgb_array")

model = PPO.load("/home/three102/hockey/logs/ppo/gymnasium_env-Hockey-v0_7/gymnasium_env-Hockey-v0.zip",env=env,device="cpu")   

vec_env = model.get_env()
obs = vec_env.reset()
for i in range(10000):
    action, _state = model.predict(obs, deterministic=False)
    obs, reward, done, info = vec_env.step(action)
    vec_env.render("human")
    if done:
      obs = vec_env.reset()
