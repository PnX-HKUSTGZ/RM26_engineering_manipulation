# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
from isaaclab.managers import SceneEntityCfg, EventTermCfg, TerminationTermCfg
from isaaclab.managers import ObservationGroupCfg, ObservationTermCfg, RewardTermCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import isaacLab.manipulation.tasks.Robot_arm.reach.mdp as mdp
import isaaclab_tasks.manager_based.manipulation.reach.mdp as general_mdp
from isaaclab_tasks.manager_based.manipulation.reach.reach_env_cfg import ReachEnvCfg

##
# Pre-defined configs
##
from isaacLab.manipulation.assets.config.version2_engineering import RM26_ENG_CFG


@configclass
class RM_Engineering_ReachEnvCfg(ReachEnvCfg):
    """RM26 Engineering Version 2 机械臂reach任务配置
    
    目标：控制右臂末端执行器到达指定位姿
    """
    
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # ========================================
        # 1. 场景配置
        # ========================================
        # 使用 RM26 V2 机器人配置
        self.scene.robot = RM26_ENG_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # 调整环境数量和间距（根据需要调整）
        self.scene.num_envs = 2048
        self.scene.env_spacing = 3.0
        # 调整地面高度，适应工程车
        self.scene.ground.init_state.pos = (0.0, 0.0, -0.2)
        
        # ========================================
        # 2. 动作配置 - 只控制右臂
        # ========================================
        self.actions.arm_action = general_mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["right_j[0-5]", "right_end"],  # 右臂6自由度 + 末端关节
            scale=0.5,  # 动作缩放系数
            use_default_offset=True,  # 使用默认位置作为偏移
        )
        # 不使用夹爪动作
        self.actions.gripper_action = None

        # ========================================
        # 3. 命令配置 - 目标位姿生成
        # ========================================
        # 指定末端执行器link
        self.commands.ee_pose.body_name = "right_end_link"
        # 调整目标位姿范围（根据机械臂工作空间调整）
        self.commands.ee_pose.ranges.pos_x = (0.3, 0.7)
        self.commands.ee_pose.ranges.pos_y = (-0.3, 0.3)
        self.commands.ee_pose.ranges.pos_z = (0.2, 0.6)
        # 姿态角度范围
        self.commands.ee_pose.ranges.roll = (-math.pi / 4, math.pi / 4)
        self.commands.ee_pose.ranges.pitch = (0.0, 0.0)  # 保持水平
        self.commands.ee_pose.ranges.yaw = (-math.pi, math.pi)
        # 目标更新频率
        self.commands.ee_pose.resampling_time_range = (5.0, 5.0)
        
        # ========================================
        # 4. 观测配置 - 强化学习输入
        # ========================================
        # 右臂关节位置 (相对于默认位置，7个关节)
        self.observations.policy.joint_pos = ObservationTermCfg(
            func=general_mdp.joint_pos_rel, 
            noise=Unoise(n_min=-0.01, n_max=0.01),
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["right_j[0-5]", "right_end"])}
        )
        # 右臂关节速度 (7个关节)
        self.observations.policy.joint_vel = ObservationTermCfg(
            func=general_mdp.joint_vel_rel, 
            noise=Unoise(n_min=-0.01, n_max=0.01),
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["right_j[0-5]", "right_end"])}
        )
        # 目标位姿命令 (7维: 3位置 + 4四元数)
        self.observations.policy.pose_command = ObservationTermCfg(
            func=general_mdp.generated_commands, 
            params={"command_name": "ee_pose"}
        )
        # 上一步动作 (7维)
        self.observations.policy.actions = ObservationTermCfg(
            func=general_mdp.last_action
        )

        # ========================================
        # 5. 奖励配置
        # ========================================
        # 位置跟踪误差惩罚（L2范数）
        self.rewards.end_effector_position_tracking = RewardTermCfg(
            func=mdp.position_command_error,
            weight=-0.5,  # 增加权重，更重视位置精度
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=["right_end_link"]), 
                "command_name": "ee_pose"
            }
        )
        # 位置跟踪奖励（tanh核函数，接近目标时奖励增大）
        self.rewards.end_effector_position_tracking_fine_grained = RewardTermCfg(
            func=mdp.position_command_error_tanh,
            weight=1.0,  # 正奖励
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=["right_end_link"]), 
                "std": 0.1,  # tanh核函数的标准差
                "command_name": "ee_pose"
            }
        )
        # 姿态跟踪误差惩罚（四元数误差）
        self.rewards.end_effector_orientation_tracking = RewardTermCfg(
            func=mdp.orientation_command_error,
            weight=-0.2,
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=["right_end_link"]), 
                "command_name": "ee_pose"
            }
        )
        # 动作变化率惩罚（平滑动作）
        self.rewards.action_rate = RewardTermCfg(
            func=general_mdp.action_rate_l2, 
            weight=-0.001
        )
        # 关节速度惩罚（避免过快运动）
        self.rewards.joint_vel = RewardTermCfg(
            func=general_mdp.joint_vel_l2,
            weight=-0.0005,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["right_j[0-5]", "right_end"])}
        )
        
        # ========================================
        # 6. 终止条件配置
        # ========================================
        # 超时终止（基于episode_length_s）
        self.terminations.time_out = TerminationTermCfg(
            func=general_mdp.time_out, 
            time_out=True
        )
        
        # ========================================
        # 7. 事件配置 - 环境重置
        # ========================================
        # 重置时随机化右臂关节位置
        self.events.reset_robot_joints = EventTermCfg(
            func=mdp.reset_joints_by_scale,
            mode="reset",
            params={
                "position_range": (0.5, 1.5),  # 相对于默认位置的缩放范围
                "velocity_range": (0.0, 0.0),   # 速度重置为0
                "asset_cfg": SceneEntityCfg("robot", joint_names=["right_j[0-5]", "right_end"])
            },
        )
        
        # ========================================
        # 8. 环境参数调整
        # ========================================
        # 控制频率：60Hz仿真，30Hz控制
        self.decimation = 2
        self.sim.dt = 1.0 / 60.0
        self.sim.render_interval = self.decimation
        
        # episode时长
        self.episode_length_s = 10.0
        
        # 相机视角
        self.viewer.eye = (3.0, 3.0, 2.5)
        self.viewer.lookat = (0.0, 0.0, 0.5)


@configclass
class RM_Engineering_ReachEnvCfg_PLAY(RM_Engineering_ReachEnvCfg):
    """用于测试/演示的配置（较少环境数量，无噪声）"""
    
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        
        # 减少环境数量用于可视化
        self.scene.num_envs = 50
        self.scene.env_spacing = 3.0
        
        # 禁用观测噪声
        self.observations.policy.enable_corruption = False
