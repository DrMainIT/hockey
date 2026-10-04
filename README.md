# PPO Air Hockey

A reinforcement learning project for a custom MuJoCo air hockey environment. The policy is trained with Proximal Policy Optimization (PPO) using Stable-Baselines3 and a Gymnasium environment.


## What is here

- `gymnasium_env/`: Gymnasium environment registration and custom wrappers.
- `assets/custom/`: MuJoCo table, striker, and robot model assets.
- `train.py`, `evaluate.py`: reproducible PPO training and interactive evaluation entry points.
- `results/models/ppo_hockey_1m.zip`: saved checkpoint from the 1M timestep experiment.
- `ppo_policy.onnx` and `ppo_policy_weights.npy`: earlier exported policy representations retained from the project.
- `results/evaluations/`: evaluation archive and a Monitor CSV from the experiment.
- `results/ppo_hockey_1m.tfevents`: original TensorBoard event data.
- `render_tensorboard.py`: exports scalar series from the event file to PNG plots.

## Environment

The observation is the concatenation of five positions (`qpos`) and five velocities (`qvel`): puck x/y/yaw, mallet x/y, then their corresponding velocities. Actions are the MuJoCo control inputs. Reset randomizes the puck position on the agent's side of the table.

The reward encourages the mallet to reach the puck, gives a bonus on contact, then encourages puck movement toward the goal. A further bonus is awarded when the puck reaches the goal region. Episodes also end when the puck leaves the valid goal lane or crosses the far boundary.

## Install

Python 3.10+ is recommended. From this directory:

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\\Scripts\\activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install -e .
```

## Train

```bash
python train.py --timesteps 1000000 --envs 5 --seed 0
```

Training writes the model, TensorBoard events, and periodic evaluation data below `results/`. View live training scalars with:

```bash
tensorboard --logdir results/tensorboard
```

## Evaluate a saved checkpoint

```bash
python evaluate.py
```

Or pass another Stable-Baselines3 checkpoint:

```bash
python evaluate.py path/to/model.zip --episodes 10 --stochastic
```

## Export TensorBoard charts

```bash
python render_tensorboard.py
```

This writes scalar charts to `results/figures/`. The saved event file is preserved in the repository so charts can be regenerated.

## Recovered training progress

The project includes eight Stable-Baselines3 run folders from `gymnasium_env-Hockey-v0_1` through `_8`. The later runs record `MlpPolicy`, 5 environments, and a configured budget of 1,000,000 timesteps. The retained `PPO_1` TensorBoard event file and checkpoint correspond to the recovered long run. Evaluation rewards varied between runs, so the checkpoint is presented as an experimental result rather than a validated or competition-ready agent.

[Download the demo video](media/video.mp4)

## Hardware experiments

`stepperControl/` contains early joystick and stepper motor experiments for physical hardware. They are separate from the MuJoCo training path and may require device-specific wiring and drivers.
