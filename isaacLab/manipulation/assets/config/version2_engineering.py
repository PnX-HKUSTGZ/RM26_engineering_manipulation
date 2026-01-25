# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the RM Engineering Version 2 robot.

The following configuration parameters are available:

* :obj:`RM_ENGINEERING_V2_CFG`: The RM Engineering Version 2 robot.
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
import os

# Path handling


BASE_DIR = os.path.dirname(os.path.abspath(__file__))

usd_dir_path = os.path.abspath(os.path.join(BASE_DIR, "../usd/rm26_version2_engineering_model"))
robot_usd_file = "rm26_version2_engineering_model.usd"
usd_path = os.path.join(usd_dir_path, robot_usd_file)

print(f"[INFO] RM26 V2 USD Path: {usd_path}")

##
# Configuration
##

RM26_ENG_CFG = ArticulationCfg(
spawn=sim_utils.UsdFileCfg(
        usd_path=usd_path,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, 
            solver_position_iteration_count=4, 
            solver_velocity_iteration_count=0,
            fix_root_link=True,
        ),
    ),
    prim_path="{ENV_REGEX_NS}/Robot",

 # --- initial state ---
init_state=ArticulationCfg.InitialStateCfg(
        # 位置：工程车初始位置，z轴稍微抬高避免穿透
        pos=(0.0, 0.0, 0.15),
        joint_pos={
            # wheels - 轮子初始角度为0
            "fl_joint": 0.0,
            "fr_joint": 0.0,
            "rl_joint": 0.0,
            "rr_joint": 0.0,

            # lift - 升降平台初始高度
            "lift_platform": 0.0,

            # left arm - 左臂初始位姿（收起状态）
            "left_j0": 0.0, "left_j1": -0.5, "left_j2": 0.5,
            "left_j3": 0.0, "left_j4": 0.5, "left_j5": 0.0,
            "left_end": 0.0, "left_end_left": 0.0, "left_end_right": 0.0,

            # right arm - 右臂初始位姿（准备工作状态）
            "right_j0": 0.0, "right_j1": -0.785, "right_j2": 0.785,
            "right_j3": 0.0, "right_j4": 0.785, "right_j5": 0.0,
            "right_end": 0.0, "right_end_left": 0.02, "right_end_right": 0.02,
        },
    ),

    # --- actuators ---
actuators={
        # 1) wheels: 轮子速度控制（本任务中不使用）
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=["fl_joint", "fr_joint", "rl_joint", "rr_joint"],
            effort_limit=50.0,
            velocity_limit=10.0,
            stiffness=0.0,
            damping=10.0,
        ),

        # 2) lift prismatic：升降平台（本任务中不使用）
        "lift": ImplicitActuatorCfg(
            joint_names_expr=["lift_platform"],
            effort_limit=500.0,
            velocity_limit=0.5,
            stiffness=0.0,
            damping=50.0,
        ),

        # 3) left arm joints（不含夹爪）- 本任务不使用
        "left_arm": ImplicitActuatorCfg(
            joint_names_expr=["left_j[0-5]", "left_end"],
            effort_limit=87.0,
            velocity_limit=2.0,
            stiffness=400.0,
            damping=80.0,
        ),

        # 4) right arm joints（不含夹爪）- 主要控制对象
        "right_arm": ImplicitActuatorCfg(
            joint_names_expr=["right_j[0-5]", "right_end"],
            effort_limit=87.0,
            velocity_limit=2.0,
            stiffness=400.0,
            damping=80.0,
        ),

        # 5) left gripper：左手夹爪
        "left_gripper": ImplicitActuatorCfg(
            joint_names_expr=["left_end_left", "left_end_right"],
            effort_limit=200.0,
            velocity_limit=0.2,
            stiffness=2000.0,
            damping=100.0,
        ),
        
        # 6) right gripper：右手夹爪
        "right_gripper": ImplicitActuatorCfg(
            joint_names_expr=["right_end_left", "right_end_right"],
            effort_limit=200.0,
            velocity_limit=0.2,
            stiffness=2000.0,
            damping=100.0,
        ),
    },
)