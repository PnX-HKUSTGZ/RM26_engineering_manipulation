# ...no filepath: user decides where to place this file...

# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import os

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import isaacLab.manipulation.tasks.Robot_arm.reach.mdp as mdp
import isaaclab_tasks.manager_based.manipulation.reach.mdp as general_mdp

##
# Pre-defined configs
##
from isaacLab.manipulation.assets.config.version2_engineering import RM26_ENG_CFG

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CUSTOM_USD_DIR = os.path.abspath(os.path.join(BASE_DIR, "../../../../../assets/usd/custom"))
RM26_WITH_PEG_USD = os.path.join(CUSTOM_USD_DIR, "rm26_version2_engineering_model_with_peg.usda")
TARGET_POST_USD = os.path.join(CUSTOM_USD_DIR, "target_post.usda")

PEG_LENGTH_M = 0.15
TARGET_LENGTH_M = 0.10


# -----------------------------------------------------------------------------
# --- Scene ---
# -----------------------------------------------------------------------------

@configclass
class RobotSceneCfg(InteractiveSceneCfg):
    """Scene configuration for RM26 engineering reach task (no terrain/teacher)."""

    # Robot
    robot: ArticulationCfg = RM26_ENG_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=RM26_WITH_PEG_USD,
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
    )

    # Target post (static)
    target_post: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/TargetPost",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.2, -0.2, 0.8),
            rot=(0.7071068, 0.0, -0.7071068, 0.0),  # +Z -> -X
        ),
        spawn=sim_utils.UsdFileCfg(
            usd_path=TARGET_POST_USD,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=1.0,
            ),
        ),
    )

    # Simple ground (keep it minimal; adjust z to fit your platform)
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.2)),
    )

    # Optional light (safe default)
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(intensity=750.0),
    )


# -----------------------------------------------------------------------------
# --- Events ---
# -----------------------------------------------------------------------------

@configclass
class EventCfg:
    """Event configuration (startup/reset/interval)."""

    # Reset: randomize right arm joints (migrated from your original cfg)
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.5, 1.5),  # relative scale around default
            "velocity_range": (0.0, 0.0),
            "asset_cfg": SceneEntityCfg("robot", joint_names=["right_j[0-5]", "right_end"]),
        },
    )


# -----------------------------------------------------------------------------
# --- Commands ---
# -----------------------------------------------------------------------------

@configclass
class CommandsCfg:
    """Command specifications for the MDP (unused for insertion task)."""
    pass


# -----------------------------------------------------------------------------
# --- Actions ---
# -----------------------------------------------------------------------------

@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    # Only control right arm joints
    arm_action = general_mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["right_j[0-5]", "right_end"],  # 6-DoF + end joint
        scale=0.5,
        use_default_offset=True,
    )

    # No gripper in this task
    gripper_action = None


# -----------------------------------------------------------------------------
# --- Observations ---
# -----------------------------------------------------------------------------
    
@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # Right arm joint positions (relative)
        joint_pos = ObsTerm(
            func=general_mdp.joint_pos_rel,
            noise=Unoise(n_min=-0.01, n_max=0.01),
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["right_j[0-5]", "right_end"])},
        )

        # Right arm joint velocities (relative)
        joint_vel = ObsTerm(
            func=general_mdp.joint_vel_rel,
            noise=Unoise(n_min=-0.01, n_max=0.01),
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["right_j[0-5]", "right_end"])},
        )

        # Pose command (typically 7D: pos(3) + quat(4), depending on command implementation)
        ee_pose = ObsTerm(
            func=mdp.ee_pose_w,
            params={"asset_cfg": SceneEntityCfg("robot", body_names=["right_end_link"])},
        )

        target_pose = ObsTerm(
            func=mdp.target_pose_w,
            params={"asset_cfg": SceneEntityCfg("target_post", body_names="TargetPost")},
        )

        # Last action
        last_action = ObsTerm(func=general_mdp.last_action)

        def __post_init__(self):
            # Keep consistent with velocity_env_cfg style knobs
            self.history_length = 1
            self.enable_corruption = True
            self.concatenate_terms = True
            self.flatten_history_dim = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


# -----------------------------------------------------------------------------
# --- Rewards ---
# -----------------------------------------------------------------------------

@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    axis_alignment = RewTerm(
        func=mdp.axis_alignment_error,
        weight=-1.0,
        params={
            "ee_cfg": SceneEntityCfg("robot", body_names=["right_end_link"]),
            "target_cfg": SceneEntityCfg("target_post", body_names="TargetPost"),
        },
    )

    radial_distance = RewTerm(
        func=mdp.radial_distance_to_axis,
        weight=-2.0,
        params={
            "ee_cfg": SceneEntityCfg("robot", body_names=["right_end_link"]),
            "target_cfg": SceneEntityCfg("target_post", body_names="TargetPost"),
        },
    )

    insertion_depth = RewTerm(
        func=mdp.insertion_depth,
        weight=3.0,
        params={
            "ee_cfg": SceneEntityCfg("robot", body_names=["right_end_link"]),
            "target_cfg": SceneEntityCfg("target_post", body_names="TargetPost"),
            "peg_length": PEG_LENGTH_M,
            "target_length": TARGET_LENGTH_M,
        },
    )

    insertion_success_bonus = RewTerm(
        func=mdp.insertion_success,
        weight=10.0,
        params={
            "ee_cfg": SceneEntityCfg("robot", body_names=["right_end_link"]),
            "target_cfg": SceneEntityCfg("target_post", body_names="TargetPost"),
            "peg_length": PEG_LENGTH_M,
            "target_length": TARGET_LENGTH_M,
            "max_angle_deg": 5.0,
            "max_radial": 0.003,
            "min_depth": 0.06,
        },
    )

    action_rate = RewTerm(func=general_mdp.action_rate_l2, weight=-0.001)

    joint_vel = RewTerm(
        func=general_mdp.joint_vel_l2,
        weight=-0.0005,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["right_j[0-5]", "right_end"])},
    )


# -----------------------------------------------------------------------------
# --- Terminations ---
# -----------------------------------------------------------------------------

@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=general_mdp.time_out, time_out=True)

    insertion_success = DoneTerm(
        func=mdp.insertion_success,
        params={
            "ee_cfg": SceneEntityCfg("robot", body_names=["right_end_link"]),
            "target_cfg": SceneEntityCfg("target_post", body_names="TargetPost"),
            "peg_length": PEG_LENGTH_M,
            "target_length": TARGET_LENGTH_M,
            "max_angle_deg": 5.0,
            "max_radial": 0.003,
            "min_depth": 0.06,
        },
    )


# -----------------------------------------------------------------------------
# --- Curriculum ---
# -----------------------------------------------------------------------------

@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP (unused for this task)."""

    # Intentionally left empty (keep class for consistent architecture)
    pass


# -----------------------------------------------------------------------------
# --- Environment ---
# -----------------------------------------------------------------------------

@configclass
class RobotEnvCfg(ManagerBasedRLEnvCfg):
    """RM26 Engineering reach task env cfg (velocity_env_cfg-like organization)."""

    # Scene settings
    scene: RobotSceneCfg = RobotSceneCfg(num_envs=2048, env_spacing=3.0)

    # MDP settings
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        super().__post_init__()

        # --- Control / simulation ---
        self.decimation = 2
        self.sim.dt = 1.0 / 60.0
        self.sim.render_interval = self.decimation

        # Episode length
        self.episode_length_s = 10.0

        # Viewer
        self.viewer.eye = (3.0, 3.0, 2.5)
        self.viewer.lookat = (0.0, 0.0, 0.5)


# -----------------------------------------------------------------------------
# --- Play / Demo ---
# -----------------------------------------------------------------------------

@configclass
class RobotPlayEnvCfg(RobotEnvCfg):
    """Play config: fewer envs, no observation corruption."""

    def __post_init__(self):
        super().__post_init__()

        self.scene.num_envs = 50
        self.scene.env_spacing = 3.0

        # Disable observation corruption/noise (keeps terms the same)
        self.observations.policy.enable_corruption = False
