# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

import isaaclab.utils.math as math_utils
from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import combine_frame_transforms, quat_error_magnitude, quat_mul

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def position_command_error(env: ManagerBasedRLEnv, command_name: str, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize tracking of the position error using L2-norm."""
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(asset.data.root_state_w[:, :3], asset.data.root_state_w[:, 3:7], des_pos_b)
    curr_pos_w = asset.data.body_state_w[:, asset_cfg.body_ids[0], :3]  # type: ignore
    return torch.norm(curr_pos_w - des_pos_w, dim=1)


def position_command_error_tanh(
    env: ManagerBasedRLEnv, std: float, command_name: str, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Reward tracking of the position using the tanh kernel."""
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(asset.data.root_state_w[:, :3], asset.data.root_state_w[:, 3:7], des_pos_b)
    curr_pos_w = asset.data.body_state_w[:, asset_cfg.body_ids[0], :3]  # type: ignore
    distance = torch.norm(curr_pos_w - des_pos_w, dim=1)
    return 1 - torch.tanh(distance / std)


def orientation_command_error(env: ManagerBasedRLEnv, command_name: str, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize tracking orientation error using shortest path."""
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    des_quat_b = command[:, 3:7]
    des_quat_w = quat_mul(asset.data.root_state_w[:, 3:7], des_quat_b)
    curr_quat_w = asset.data.body_state_w[:, asset_cfg.body_ids[0], 3:7]  # type: ignore
    return quat_error_magnitude(curr_quat_w, des_quat_w)


def axis_alignment_error(
    env: ManagerBasedRLEnv, ee_cfg: SceneEntityCfg, target_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Angle error between end-effector axis and target post axis (radians)."""
    ee_asset = env.scene[ee_cfg.name]
    target_asset: RigidObject = env.scene[target_cfg.name]

    ee_quat_w = ee_asset.data.body_state_w[:, ee_cfg.body_ids[0], 3:7]  # type: ignore
    target_quat_w = target_asset.data.root_state_w[:, 3:7]

    z_axis = torch.tensor([0.0, 0.0, 1.0], device=env.device).repeat((ee_quat_w.shape[0], 1))
    ee_axis_w = math_utils.quat_apply(ee_quat_w, z_axis)
    target_axis_w = math_utils.quat_apply(target_quat_w, z_axis)

    ee_axis_w = ee_axis_w / (torch.norm(ee_axis_w, dim=1, keepdim=True) + 1e-8)
    target_axis_w = target_axis_w / (torch.norm(target_axis_w, dim=1, keepdim=True) + 1e-8)

    dot = torch.sum(ee_axis_w * target_axis_w, dim=1).clamp(-1.0, 1.0)
    return torch.acos(dot)


def radial_distance_to_axis(
    env: ManagerBasedRLEnv, ee_cfg: SceneEntityCfg, target_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Distance from end-effector axis origin to target post axis (meters)."""
    ee_asset = env.scene[ee_cfg.name]
    target_asset: RigidObject = env.scene[target_cfg.name]

    ee_pos_w = ee_asset.data.body_state_w[:, ee_cfg.body_ids[0], :3]  # type: ignore
    target_pos_w = target_asset.data.root_state_w[:, :3]
    target_quat_w = target_asset.data.root_state_w[:, 3:7]

    z_axis = torch.tensor([0.0, 0.0, 1.0], device=env.device).repeat((ee_pos_w.shape[0], 1))
    target_axis_w = math_utils.quat_apply(target_quat_w, z_axis)
    target_axis_w = target_axis_w / (torch.norm(target_axis_w, dim=1, keepdim=True) + 1e-8)

    rel = ee_pos_w - target_pos_w
    proj = torch.sum(rel * target_axis_w, dim=1, keepdim=True) * target_axis_w
    radial_vec = rel - proj
    return torch.norm(radial_vec, dim=1)


def insertion_depth(
    env: ManagerBasedRLEnv,
    ee_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    peg_length: float,
    target_length: float,
) -> torch.Tensor:
    """Compute insertion depth of the peg tip into the target post (meters)."""
    ee_asset = env.scene[ee_cfg.name]
    target_asset: RigidObject = env.scene[target_cfg.name]

    ee_pos_w = ee_asset.data.body_state_w[:, ee_cfg.body_ids[0], :3]  # type: ignore
    ee_quat_w = ee_asset.data.body_state_w[:, ee_cfg.body_ids[0], 3:7]  # type: ignore
    target_pos_w = target_asset.data.root_state_w[:, :3]
    target_quat_w = target_asset.data.root_state_w[:, 3:7]

    z_axis = torch.tensor([0.0, 0.0, 1.0], device=env.device).repeat((ee_pos_w.shape[0], 1))
    ee_axis_w = math_utils.quat_apply(ee_quat_w, z_axis)
    target_axis_w = math_utils.quat_apply(target_quat_w, z_axis)
    ee_axis_w = ee_axis_w / (torch.norm(ee_axis_w, dim=1, keepdim=True) + 1e-8)
    target_axis_w = target_axis_w / (torch.norm(target_axis_w, dim=1, keepdim=True) + 1e-8)

    peg_tip_pos = ee_pos_w + ee_axis_w * (peg_length * 0.5)
    rel = peg_tip_pos - target_pos_w
    proj = torch.sum(rel * target_axis_w, dim=1)
    depth = proj - (target_length * 0.5)
    return torch.clamp(depth, min=0.0)


def insertion_success(
    env: ManagerBasedRLEnv,
    ee_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    peg_length: float,
    target_length: float,
    max_angle_deg: float,
    max_radial: float,
    min_depth: float,
) -> torch.Tensor:
    """Binary success for insertion based on angle, radial, and depth thresholds."""
    angle = axis_alignment_error(env, ee_cfg, target_cfg)
    radial = radial_distance_to_axis(env, ee_cfg, target_cfg)
    depth = insertion_depth(env, ee_cfg, target_cfg, peg_length, target_length)

    max_angle = torch.deg2rad(torch.tensor(max_angle_deg, device=env.device))
    success = (angle < max_angle) & (radial < max_radial) & (depth > min_depth)
    return success.float()
