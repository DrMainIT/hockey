from typing import Dict, Union

import numpy as np
from icecream import ic
from gymnasium import utils
from gymnasium.envs.mujoco import MujocoEnv
from gymnasium.spaces import Box

"""
# two methods
distance_per_rev = 2 * 3.14159 * wheel_radius  # mm
distance_per_rev = pulley_teeth * belt_pitch
steps_per_mm = (steps_per_rev * microstepping) / distance_per_rev

# the action in normalized between -1 and 1 if the dimension of the table are
# 1000x500mm
x = 250 * action[0]
y = 500 * action[1]

steps_x = steps_per_mm * x
steps_y = steps_per_mm * y
"""

"""
train: python -m rl_zoo3.train --algo ppo  --env gymnasium_env/Hockey-v0 --tensorboard-log tensorboard --device cpu --save-freq 600000 -P
enjoy: python -m rl_zoo3.enjoy --algo ppo --env gymnasium_env/Hockey-v0 --exp-id 0 --folder logs -n 5000      
python -m rl_zoo3.enjoy --algo ppo --env gymnasium_env/Hockey-v0 --folder logs --exp-id 9  --load-checkpoint 1800000 -n 10000           

qpos: 
    puck_pos_x
    puck_pos_y
    puck_yaw
    mallet_pos_x
    mallet_pos_y

qvel:
    puck_vel_x
    puck_vel_y
    puck_angvel
    mallet_vel_x
    mallet_vel_y
"""


DEFAULT_CAMERA_CONFIG = {
    "trackbodyid": 0,
    "distance": 3.04,
}

class HockeyEnv(MujocoEnv, utils.EzPickle):
    metadata = {
        "render_modes": [
            "human",
            "rgb_array",
            "depth_array",
            "rgbd_tuple",
        ],
    }

    def __init__(
            self,
            xml_file: str = "/home/three102/hockey/assets/custom/custom.xml",
            frame_skip: int = 1,
            default_camera_config: Dict[str, Union[float, int]] = DEFAULT_CAMERA_CONFIG,
            reset_noise_scale: float = 0.01,
            device: str = "cpu",
            **kwargs,
    ):
        utils.EzPickle.__init__(self, xml_file, frame_skip, reset_noise_scale, **kwargs)
        observation_space = Box(low=-np.inf, high=np.inf, shape=(10,), dtype=np.float64)
        self.hit = False
        self.goal = False
        self._reset_noise_scale = reset_noise_scale
        MujocoEnv.__init__(
            self,
            xml_file,
            frame_skip,
            observation_space=observation_space,
            default_camera_config=default_camera_config,
            **kwargs,
        )

        self.metadata = {
            "render_modes": [
                "human",
                "rgb_array",
                "depth_array",
                "rgbd_tuple",
            ],
            "render_fps": int(np.round(1.0 / self.dt)),
        }
        self.observation_structure = {
            "qpos": self.data.qpos.size,
            "qvel": self.data.qvel.size,
        }

    def _hit_reward(self):  
        rew = 0             
        if self.data.ncon != 0:  # Threshold for "hitting" the puck
            for i in range(self.data.ncon):  # Iterate through all active contacts
                if self.data.contact[i].geom1 == 10 and self.data.contact[i].geom2 == 13:
                    if self.hit == False:
                        rew = 100.0
                        self.hit = True

        return rew
    
    def _goal_reward(self,puck_pos_x_normalized, puck_pos_y):
        rew = 0
        if self.goal == False and puck_pos_x_normalized >= 0.95 and abs(puck_pos_y) <= 0.1:
                rew = 100.0
                self.goal = True
        return rew

    def _distance_reward(self, puck_pos_x_normalized, puck_pos_y_normalized, mallet_pos_x_normalized, mallet_pos_y_normalized):
        rew = 0
        # Calculate the Euclidean distance between the normalized positions
        distance = np.sqrt(
            (puck_pos_x_normalized - mallet_pos_x_normalized) ** 2 +
            (puck_pos_y_normalized - mallet_pos_y_normalized) ** 2
        )
        rew = -distance
        return rew
    
    def get_reward(self, obs):
        reward = 0.0
        terminated = False
        puck_pos_x = obs[0]
        puck_pos_y = obs[1]
        mallet_pos_x = obs[3]
        mallet_pos_y = obs[4]
        # Calculate the Euclidean distance between the puck and the mallet
        puck_pos_x_normalized = (puck_pos_x + 1) / 2  # Normalize from [-1, 1] to [0, 1]
        puck_pos_y_normalized = (puck_pos_y + 0.45) / 0.9  # Normalize from [-1, 1] to [0, 1]
        mallet_pos_x_normalized = mallet_pos_x  # Normalize from [0, 0.5] to [0, 1]
        mallet_pos_y_normalized = (mallet_pos_y + 0.45) / 0.9 # Normalize from [0, 0.5] to [0, 1]
        
        # Calculate the distance reward
        distance_reward = self._distance_reward(puck_pos_x_normalized, puck_pos_y_normalized, mallet_pos_x_normalized, mallet_pos_y_normalized)
        if self.hit == False:
            reward += distance_reward
        
        # Extract positions from the observation
        hit_reward = self._hit_reward()
        goal_reward = self._goal_reward(puck_pos_x_normalized, puck_pos_y)

        reward += hit_reward  + goal_reward


        return reward
    

    def step(self, action):
        self.do_simulation(action, self.frame_skip)
        truncated = False
        terminated = False
        observation = self._get_obs()
        reward = self.get_reward(observation)
        info = {"custom": 0}

        if self.data.qpos[0] > 1.10:
            terminated = True
        #if self.data.qpos[0] < -0.90:
        #    terminated = True

        if self.render_mode == "human":
            self.render()

        return observation, reward, terminated, truncated, info
    
    
    def reset_model(self):
        self.hit = False
        self.goal = False
        self.step_count = 0
        


        qpos = self.init_qpos
        qvel = self.init_qvel
        # Add noise to the initial velocity
        #qvel[0] = -abs(np.random.randint(10, 20))
        #qvel[1] = np.random.randn()*10
        # add noise to the initial position
        qpos[0] = np.random.uniform(-0.5, 0)
        qpos[1] = np.random.uniform(-0.4, 0.4)
        self.set_state(qpos, qvel)
        return self._get_obs()

    def _get_obs(self):
        return np.concatenate([self.data.qpos, self.data.qvel]).ravel()
    
if __name__ == "__main__":
    import gymnasium as gym
    import gymnasium_env

    env = gym.make("gymnasium_env/Hockey-v0",render_mode="human")
    env = gym.wrappers.TimeLimit(env, max_episode_steps=20)
    env.reset()
    for _ in range(1000):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        env.render()
        if terminated or truncated:
            obs = env.reset()
    env.close()