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
usd_dir_path = os.path.normpath(os.path.join(BASE_DIR, "../usd/rm26_version1_engineering_model/"))
print("USD DIR PATH:", usd_dir_path)
robot_usd = "/rm26_version1_engineering_model.usd"

print("USD PATH:", usd_dir_path + robot_usd)
##
# Configuration
##

# 1. Spawn Configuration
rm_engineering_spawn_cfg = sim_utils.UsdFileCfg(
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
)

# 2. Initial State Configuration
rm_engineering_init_state_cfg = ArticulationCfg.InitialStateCfg(
    joint_pos={
        "j1": 0.0,
        "j2": 0.5,
        "j3": 0.0,
        "j4": 0.0,
        "j5": 0.0,
        "end_differential_controller/pitch_joint": 0.0,
        "end_differential_controller/roll_joint": 0.0,
        "fl_joint": 0.0,
        "fr_joint": 0.0,
        "rl_joint": 0.0,
        "rr_joint": 0.0,
    },
)

# 3. Actuators Configuration
rm_engineering_actuators_cfg = {
    "arm": ImplicitActuatorCfg(
        joint_names_expr=["j[1-5]", "end_differential_controller/.*"],
        velocity_limit=2.0,
        effort_limit=50.0,
        stiffness={
            "j[1-2]": 200.0,
            "j[3-5]": 150.0,
            "end_differential_controller/.*": 50.0,
        },
        damping={
            "j[1-2]": 10.0,
            "j[3-5]": 5.0,
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
}

print("RM Engineering Actuators Config:", rm_engineering_actuators_cfg)
# 4. Final Articulation Configuration
RM_ENGINEERING_ARM = ArticulationCfg(
    spawn=rm_engineering_spawn_cfg,
    init_state=rm_engineering_init_state_cfg,
    actuators=rm_engineering_actuators_cfg,
)

print("RM_ENGINEERING_ARM:", RM_ENGINEERING_ARM)


