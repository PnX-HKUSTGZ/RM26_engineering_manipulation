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
            disable_gravity=True,
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
            solver_velocity_iteration_count=0
        ),
    ),
    prim_path="{ENV_REGEX_NS}/Robot",

 # --- initial state ---
init_state=ArticulationCfg.InitialStateCfg(
        # 位置你按场景调；工程车一般 z 稍微抬一点避免初始穿透
        pos=(0.0, 0.0, 0.20),
        joint_pos={
            # wheels
            "fl_joint": 0.0,
            "fr_joint": 0.0,
            "rl_joint": 0.0,
            "rr_joint": 0.0,

            # lift
            "lift_platform": 0.0,

            # left arm
            "left_j0": 0.0, "left_j1": 0.0, "left_j2": 0.0,
            "left_j3": 0.0, "left_j4": 0.0, "left_j5": 0.0,
            "left_end": 0.0, "left_end_left": 0.0, "left_end_right": 0.0,

            # right arm
            "right_j0": 0.0, "right_j1": 0.0, "right_j2": 0.0,
            "right_j3": 0.0, "right_j4": 0.0, "right_j5": 0.0,
            "right_end": 0.0, "right_end_left": 0.0, "right_end_right": 0.0,
        },
    ),

    # --- actuators ---
actuators={
        # 1) wheels: 通常给速度/力矩都可以；这里用 implicit actuator 并显式给 limit
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=["fl_joint", "fr_joint", "rl_joint", "rr_joint"],
            effort_limit_sim=0.0,
            velocity_limit_sim=80.0,
            # 轮子很多情况下你会用 velocity drive（stiffness/damping 根据你 action 定义再调）
            stiffness=0.0,
            damping=20.0,
        ),

        # 2) lift prismatic：速度小、推力大（数值你按真实电机/丝杆再标定）
        "lift": ImplicitActuatorCfg(
            joint_names_expr=["lift_platform"],
            effort_limit_sim=0.0,
            velocity_limit_sim=1.0,
            stiffness=0.0,
            damping=20.0,
        ),

        # 3) left arm joints（不含夹爪）
        "left_arm": ImplicitActuatorCfg(
            joint_names_expr=["left_j[0-5]", "left_end"],
            effort_limit_sim=120.0,
            velocity_limit_sim=8.0,
            stiffness=0.0,
            damping=5.0,
        ),

        # 4) right arm joints（不含夹爪）
        "right_arm": ImplicitActuatorCfg(
            joint_names_expr=["right_j[0-5]", "right_end"],
            effort_limit_sim=120.0,
            velocity_limit_sim=8.0,
            stiffness=0.0,
            damping=5.0,
        ),

        # 5) grippers：末端左右夹指，建议用更强 damping（更稳）
        "left_gripper": ImplicitActuatorCfg(
            joint_names_expr=["left_end_left", "left_end_right"],
            effort_limit_sim=0.0,
            velocity_limit_sim=4.0,
            stiffness=0.0,
            damping=20.0,
        ),
        "right_gripper": ImplicitActuatorCfg(
            joint_names_expr=["right_end_left", "right_end_right"],
            effort_limit_sim=0.0,
            velocity_limit_sim=4.0,
            stiffness=0.0,
            damping=20.0,
        ),
    },
)