# ...no filepath: user decides where to place this file...

# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
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


# -----------------------------------------------------------------------------
# --- Scene ---
# -----------------------------------------------------------------------------

@configclass
class RobotSceneCfg(InteractiveSceneCfg):
    """Scene configuration for RM26 engineering reach task (no terrain/teacher)."""

    # Robot
    robot: ArticulationCfg = RM26_ENG_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

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
    """Command specifications for the MDP.

    This task uses an end-effector pose command named 'ee_pose'.
    """

    # NOTE: API compatibility guard (keeps file importable across versions).
    if hasattr(general_mdp, "UniformPoseCommandCfg"):
        ee_pose = general_mdp.UniformPoseCommandCfg(
            asset_name="robot",
            body_name="right_end_link",
            resampling_time_range=(5.0, 5.0),
            ranges=general_mdp.UniformPoseCommandCfg.Ranges(
                pos_x=(0.3, 0.7),
                pos_y=(-0.3, 0.3),
                pos_z=(0.2, 0.6),
                roll=(-math.pi / 4, math.pi / 4),
                pitch=(0.0, 0.0),  # keep level
                yaw=(-math.pi, math.pi),
            ),
        )
    else:
        # TODO: Replace with the correct pose-command cfg for your IsaacLab version.
        # For example: general_mdp.PoseCommandCfg / mdp.UniformPoseCommandCfg / etc.
        ee_pose = None


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
        pose_command = ObsTerm(
            func=general_mdp.generated_commands,
            params={"command_name": "ee_pose"},
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

    end_effector_position_tracking = RewTerm(
        func=mdp.position_command_error,
        weight=-0.5,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["right_end_link"]),
            "command_name": "ee_pose",
        },
    )

    end_effector_position_tracking_fine_grained = RewTerm(
        func=mdp.position_command_error_tanh,
        weight=1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["right_end_link"]),
            "std": 0.1,
            "command_name": "ee_pose",
        },
    )

    end_effector_orientation_tracking = RewTerm(
        func=mdp.orientation_command_error,
        weight=-0.2,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["right_end_link"]),
            "command_name": "ee_pose",
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