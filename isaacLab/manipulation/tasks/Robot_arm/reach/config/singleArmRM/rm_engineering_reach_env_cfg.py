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
from isaaclab.manipulation.assets.config.singleArmRM import RM_ENGINEERING_ARM  # 导入你的机器人配置


@configclass
class RM_Engineering_ReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # 1. 替换机器人
        self.scene.robot = RM_ENGINEERING_ARM.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # 2. 配置动作 (Action)
        # 使用 "arm" 组 (在 singleArmRM.py 中定义的 actuators["arm"])
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["j[1-5]_joint", "end_differential_controller/.*"], scale=0.5, use_default_offset=True
        )
        # 如果没有夹爪动作，可以设为 None
        self.actions.gripper_action = None

        # 3. 配置命令 (Command) - 末端执行器名称
        # 你的 URDF 中末端 link 名字是 "j7_link" (根据之前的 URDF 分析)
        # 注意：这里需要指定 body_name，用于计算 reward 和 observation
        self.commands.ee_pose.body_name = "j7_link"

        # 4. 配置奖励 (Rewards) - 同样需要指定末端 link
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["j7_link"]
        self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["j7_link"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["j7_link"]

        # 5. 调整初始位置 (可选)
        # 如果需要调整机器人相对于桌子的位置
        self.scene.robot.init_state.pos = (0.0, 0.0, 0.0) 
