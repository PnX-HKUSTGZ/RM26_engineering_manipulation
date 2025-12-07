# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the RM Engineering arm.

The following configuration parameters are available:

* :obj:`RM_ENGINEERING_ARM`: The RM Engineering arm.
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
usd_dir_path = os.path.join(BASE_DIR, "../usd/")

robot_usd = "rm_engineering_model.usd"

##
# Configuration
##

RM_ENGINEERING_ARM = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=usd_dir_path + robot_usd,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
            fix_root_link=True,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "j1_joint": 0.0,
            "j2_joint": 0.5,
            "j3_joint": 0.0,
            "j4_joint": 0.0,
            "j5_joint": 0.0,
            "end_differential_controller/pitch_joint": 0.0,
            "end_differential_controller/roll_joint": 0.0,
            "fl_joint": 0.0,
            "fr_joint": 0.0,
            "rl_joint": 0.0,
            "rr_joint": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["j[1-5]_joint", "end_differential_controller/.*"],
            velocity_limit=2.0,
            effort_limit=50.0,
            stiffness={
                "j[1-2]_joint": 200.0,
                "j[3-5]_joint": 150.0,
                "end_differential_controller/.*": 50.0,
            },
            damping={
                "j[1-2]_joint": 10.0,
                "j[3-5]_joint": 5.0,
                "end_differential_controller/.*": 1.0,
            },
        ),
        "base_wheels": ImplicitActuatorCfg(
            joint_names_expr=["[fr][lr]_joint"],
            effort_limit=100.0,
            velocity_limit=10.0,
            stiffness=0.0,
            damping=100.0,
        ),
    },
)


