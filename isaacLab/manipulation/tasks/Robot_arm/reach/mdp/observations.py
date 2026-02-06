# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Observation helpers for the RM26 insertion task."""

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def ee_pose_w(env: "ManagerBasedRLEnv", asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """End-effector pose in world frame as (x, y, z, w, x, y, z)."""
    asset = env.scene[asset_cfg.name]
    return asset.data.body_state_w[:, asset_cfg.body_ids[0], :7]  # type: ignore


def target_pose_w(env: "ManagerBasedRLEnv", asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Target post pose in world frame as (x, y, z, w, x, y, z)."""
    asset: RigidObject = env.scene[asset_cfg.name]
    return asset.data.root_state_w[:, :7]
