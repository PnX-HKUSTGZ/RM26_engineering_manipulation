# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents, rm_engineering_reach_env_cfg, rsl_rl_ppo_cfg

##
# Register Gym environments.
##

##
# RM26 Engineering V2 - Joint Position Control
##

gym.register(
    id="Template-Isaac-Reach-RM26-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": rm_engineering_reach_env_cfg.RobotEnvCfg,
        "rsl_rl_cfg_entry_point": rsl_rl_ppo_cfg.EngineerReachPPORunnerCfg,
    },
)

gym.register(
    id="Template-Isaac-Reach-RM26-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": rm_engineering_reach_env_cfg.RobotPlayEnvCfg,
        "rsl_rl_cfg_entry_point": rsl_rl_ppo_cfg.EngineerReachPPORunnerCfg,
    },
)
