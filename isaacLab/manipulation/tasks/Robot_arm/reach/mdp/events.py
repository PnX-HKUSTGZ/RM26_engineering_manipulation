# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Functions specific to the in-hand dexterous manipulation environments."""


from __future__ import annotations

import torch
from typing import TYPE_CHECKING, Literal

from isaaclab.assets import Articulation
from isaaclab.managers import EventTermCfg, ManagerTermBase, SceneEntityCfg
from isaaclab.utils.math import sample_uniform

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


#class reset_joint(ManagerTermBase):
def reset_joints_by_scale(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    position_range: tuple[float, float],
    velocity_range: tuple[float, float],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    """Reset the robot joints by scaling the default joint positions.

    The function samples a random scale from `position_range` and multiplies the default joint positions
    by this scale. The velocities are sampled from `velocity_range`.

    Args:
        env: The environment.
        env_ids: The environment ids.
        position_range: The range of the scale for the joint positions.
        velocity_range: The range of the velocity for the joint velocities.
        asset_cfg: The configuration of the asset.
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]

    # resolve keys
    if asset_cfg.joint_ids is None:
        joint_ids = slice(None)
    else:
        joint_ids = asset_cfg.joint_ids

    # get default joint positions
    # shape: (num_envs, num_selected_joints)
    if isinstance(joint_ids, slice):
        joint_pos = asset.data.default_joint_pos[env_ids, joint_ids].clone()
        joint_vel = asset.data.default_joint_vel[env_ids, joint_ids].clone()
    else:
        # Avoid advanced indexing shape mismatch [N] vs [M]
        joint_pos = asset.data.default_joint_pos[env_ids][:, joint_ids].clone()
        joint_vel = asset.data.default_joint_vel[env_ids][:, joint_ids].clone()

    # scale positions
    scale = sample_uniform(position_range[0], position_range[1], joint_pos.shape, device=env.device)
    joint_pos[:] = joint_pos * scale

    # scale velocities
    joint_vel[:] = sample_uniform(velocity_range[0], velocity_range[1], joint_vel.shape, device=env.device)

    # set into the physics simulation
    asset.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids, joint_ids=joint_ids)


