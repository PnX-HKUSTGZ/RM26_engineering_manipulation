# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Termination helpers for the RM26 insertion task."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.managers import SceneEntityCfg

from .rewards import axis_alignment_error, insertion_depth, radial_distance_to_axis

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def insertion_success(
    env: "ManagerBasedRLEnv",
    ee_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    peg_length: float,
    target_length: float,
    max_angle_deg: float,
    max_radial: float,
    min_depth: float,
) -> torch.Tensor:
    """Terminate when insertion metrics exceed thresholds."""
    angle = axis_alignment_error(env, ee_cfg, target_cfg)
    radial = radial_distance_to_axis(env, ee_cfg, target_cfg)
    depth = insertion_depth(env, ee_cfg, target_cfg, peg_length, target_length)

    max_angle = torch.deg2rad(torch.tensor(max_angle_deg, device=env.device))
    success = (angle < max_angle) & (radial < max_radial) & (depth > min_depth)
    return success
