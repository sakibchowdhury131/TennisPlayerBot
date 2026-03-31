# TennisPlayerBot

A robotic table tennis player built with PyBullet simulation, MoveIt motion planning, and machine learning. The system uses a Franka Emika Panda arm in simulation to hit a table tennis ball, combining a physics-based PyBullet Gym environment with MoveIt-based trajectory planning for real robot deployment.

## Features

- PyBullet simulation environment for table tennis (Gym-compatible, versions v1–v4)
- PPO reinforcement learning agent for hitting strategy (`ppo-panda-tennis-model.model`)
- MoveIt ROS packages for motion planning and real-robot execution
- Ball trajectory prediction and URDF model customization
- RealSense depth camera integration for ball tracking
- Custom URDF models for the table tennis setup

## Tech Stack

- Python 3
- PyBullet
- OpenAI Gym
- ROS (MoveIt)
- Stable Baselines3 / PyTorch (PPO)
- Intel RealSense SDK
- NumPy, Pandas

## Project Structure

| File / Directory | Description |
|---|---|
| `gym_environment-table-tennis-v*.ipynb` | Gym environment iterations (v1–v4) |
| `ball-tracking.ipynb` | Ball tracking using RealSense camera |
| `URDF-editor.ipynb` | URDF model editing notebook |
| `moveitPackages/` | ROS MoveIt packages for arm control |
| `modURDFs/` | Modified URDF files |
| `BallLauncherControl/` | Ball launcher hardware controller |
| `cameraControl/` | Camera control utilities |
| `delta_estimator_mlp/` | MLP for delta position estimation |
| `ppo-panda-tennis-model.model` | Trained PPO model |
| `ball_trajectory_data*.csv` | Ball trajectory datasets |

## Requirements

- ROS Noetic (for MoveIt components)
- PyBullet, Gym, NumPy, Pandas
- Intel RealSense SDK (for camera features)

## License

MIT
