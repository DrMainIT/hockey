"""Run a trained PPO policy in the interactive MuJoCo viewer."""
import argparse
from pathlib import Path

import gymnasium as gym
import gymnasium_env  # noqa: F401 - registers Hockey-v0
from stable_baselines3 import PPO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("model", nargs="?", type=Path, default=Path("results/models/ppo_hockey_1m.zip"))
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--stochastic", action="store_true")
    args = parser.parse_args()
    env = gym.make("gymnasium_env/Hockey-v0", render_mode="human")
    model = PPO.load(args.model, env=env, device="cpu")
    obs, _ = env.reset()
    episodes = 0
    while episodes < args.episodes:
        action, _ = model.predict(obs, deterministic=not args.stochastic)
        obs, _, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            obs, _ = env.reset()
            episodes += 1
    env.close()


if __name__ == "__main__":
    main()
