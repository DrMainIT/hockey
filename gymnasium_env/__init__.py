from gymnasium.envs.registration import register

register(
    id="gymnasium_env/Hockey-v0",
    entry_point="gymnasium_env.envs:HockeyEnv",
    max_episode_steps=1000,
    reward_threshold=950.0,
)