import gymnasium as gym
import gymnasium_env
from stable_baselines3 import PPO

env = gym.make("gymnasium_env/Hockey-v0", render_mode="rgb_array")
env = gym.wrappers.TimeLimit(env, max_episode_steps=200)

model = PPO("MlpPolicy",env,verbose=1)
print(model.policy)

model.learn(total_timesteps=50_000)

#model.save("ppo_hockey1")

vec_env = model.get_env()

obs = vec_env.reset()
for i in range(1000):
    action, _state = model.predict(obs, deterministic=False)
    obs, reward, done, info = vec_env.step(action)
    vec_env.render("human")
    if done:
        obs = vec_env.reset()
