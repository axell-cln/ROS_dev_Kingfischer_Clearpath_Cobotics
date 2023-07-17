#!/home/kingfisher/venv/rl/bin/python
from gym import spaces
import numpy as np
import torch
import yaml
import pandas as pd
from rl_games.algos_torch.players import BasicPpoPlayerContinuous


config_name = "/home/kingfisher/ros_ws/src/test_rl/config/TurtlebotPPO.yaml"
policy_path="/home/kingfisher/ros_ws/src/test_rl/config/Turtlebot.pth"


with open(config_name, 'r') as stream:
    cfg = yaml.safe_load(stream)

observation_space = spaces.Box(np.ones(2) * -np.Inf, np.ones(2) * np.Inf)

act_space = spaces.Box(low=np.array([0.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)


player = BasicPpoPlayerContinuous(cfg, observation_space, act_space, clip_actions=True, deterministic=True, device="cpu")
player.restore(policy_path)


for i in range(1):
    obs = dict({'obs':torch.tensor([1.0,0.5], dtype=torch.float32, device='cpu')})
    action = player.get_action(obs["obs"], is_deterministic=True)
    print("itteration: ", i)
    print("obs: ",obs)
    print("actions: ",action)
    print("")

