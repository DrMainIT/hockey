"""Train a PPO agent in the custom MuJoCo air hockey environment."""
import argparse
from pathlib import Path

import gymnasium as gym
import gymnasium_env  # noqa: F401 - registers Hockey-v0
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv


def make_env(render_mode=None):
    return gym.make("gymnasium_env/Hockey-v0", render_mode=render_mode)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=1_000_000)
    parser.add_argument("--envs", type=int, default=5)
    parser.add_argument("--output", type=Path, default=Path("results"))
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    vec_cls = DummyVecEnv if args.envs == 1 else SubprocVecEnv
    env = vec_cls([lambda: make_env() for _ in range(args.envs)])
    eval_env = DummyVecEnv([lambda: make_env()])
    callback = EvalCallback(
        eval_env,
        best_model_save_path=str(args.output / "best_model"),
        log_path=str(args.output / "evaluations"),
        eval_freq=max(25_000 // args.envs, 1),
        n_eval_episodes=5,
        deterministic=True,
    )
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=str(args.output / "tensorboard"), seed=args.seed)
    model.learn(total_timesteps=args.timesteps, callback=callback, tb_log_name="PPO")
    model.save(args.output / "ppo_hockey")
    env.close()
    eval_env.close()


if __name__ == "__main__":
    main()
