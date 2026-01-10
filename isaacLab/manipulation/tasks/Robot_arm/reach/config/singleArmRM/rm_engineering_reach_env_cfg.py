# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import isaacLab.manipulation.tasks.Robot_arm.reach.mdp as mdp
import isaaclab_tasks.manager_based.manipulation.reach.mdp as general_mdp
from isaaclab_tasks.manager_based.manipulation.reach.reach_env_cfg import ReachEnvCfg

##
# Pre-defined configs
##
from isaacLab.manipulation.assets.config.version2_engineering import RM26_ENG_CFG


@configclass
class RM_Engineering_ReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # 1. 使用 V2 机器人配置
        self.scene.robot = RM26_ENG_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # 2. 配置动作 (只控制右臂关节)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["right_j[0-5]", "right_end"],
            scale=0.5,
            use_default_offset=True,
        )
        self.actions.gripper_action = None

        # 3. 指定末端执行器 link
        self.commands.ee_pose.body_name = "right_end_link"

        # 4. 奖励项绑定末端 link
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["right_end_link"]
        self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["right_end_link"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["right_end_link"]
