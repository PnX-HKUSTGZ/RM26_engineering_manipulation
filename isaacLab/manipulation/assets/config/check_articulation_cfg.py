# check_articulation_cfg.py
# ------------------------------------------------------------
# IsaacLab ArticulationCfg Sanity Checker
# ------------------------------------------------------------

from isaaclab.app import AppLauncher

# ------------------------------------------------------------
# 1. Launch Isaac Sim
# ------------------------------------------------------------
# ⚠️ MUST be called before importing any other isaaclab modules that depend on Isaac Sim
app_launcher = AppLauncher(headless=False)
simulation_app = app_launcher.app

# ------------------------------------------------------------
# 2. Import Isaac Lab modules & your ArticulationCfg
# ------------------------------------------------------------
import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation

# ⚠️ 改成你自己的 cfg
from version2_engineering import RM26_ENG_CFG


# ------------------------------------------------------------
# 3. Create Simulation Context
# ------------------------------------------------------------
sim = sim_utils.SimulationContext(
    sim_utils.SimulationCfg(
        dt=1.0 / 120.0,
        device="cuda",
    )
)

sim.reset()

# ------------------------------------------------------------
# 4. Spawn Robot
# ------------------------------------------------------------
RM26_ENG_CFG.prim_path = "/World/Robot"
RM26_ENG_CFG.spawn.func(RM26_ENG_CFG.prim_path, RM26_ENG_CFG.spawn)
robot = Articulation(cfg=RM26_ENG_CFG)


sim.reset()

print("\n==============================")
print(" Articulation Spawned ")
print("==============================\n")

# ------------------------------------------------------------
# 5. Joint Name Check
# ------------------------------------------------------------
print(">>> Joint Names")
print("--------------------------------")
print(robot.joint_names)
print(f"Total joints: {robot.num_joints}\n")

# ------------------------------------------------------------
# 6. Actuator Binding Check
# ------------------------------------------------------------
print(">>> Actuator Bindings")
print("--------------------------------")
for name, actuator in robot.actuators.items():
    jnames = actuator.joint_names
    print(f"[{name}] -> {jnames}")
    if len(jnames) == 0:
        print("  ⚠️  WARNING: actuator has NO joints bound!")
print()

# ------------------------------------------------------------
# 7. Zero-Action Stability Test
# ------------------------------------------------------------
print(">>> Zero-Action Stability Test")
print("--------------------------------")

zero_action = torch.zeros((robot.num_instances, robot.num_joints), device=sim.device)

# 还需要导入 ArticulationActions 来应用动作
from isaaclab.assets import ArticulationActions

for i in range(300):
    # ⚠️ 关键：不能直接传 tensor，需要封装成 ArticulationActions
    # 这里我们发送 0 力矩 (Effort)，模拟被动状态
    action = ArticulationActions(joint_efforts=zero_action)
    robot.apply_action(action)
    
    # 更新物理步
    sim.step()

q = robot.data.joint_pos.cpu()
qd = robot.data.joint_vel.cpu()

print("Joint position (last step):")
print(q)

print("\nJoint velocity (last step):")
print(qd)

print("\nIf velocities explode or NaN → actuator / inertia problem\n")

# ------------------------------------------------------------
# 8. Single-Joint Excitation Test
# ------------------------------------------------------------
print(">>> Single-Joint Excitation Test")
print("--------------------------------")

if robot.num_joints == 0:
    print("❌ No joints available. Check actuator configuration.")
else:
    # 1. 准备动作张量 (num_envs, num_joints)
    test_effort = torch.zeros((robot.num_instances, robot.num_joints), device=sim.device)
    
    # 2. 找到 "left_j1" 关节的索引 (假设你想动左臂的第二个关节)
    # 你可以根据 robot.joint_names 打印出来的列表来确认名字
    target_joint_name = "left_j1" 
    
    try:
        joint_idx = robot.joint_names.index(target_joint_name)
        print(f"Exciting joint: {target_joint_name} (Index: {joint_idx})")
        
        # 3. 施加一个小力矩
        test_effort[:, joint_idx] = 10.0  # 10 Nm，根据你的动力学调整大小

        for i in range(240):
            # 4. 应用动作
            action = ArticulationActions(joint_efforts=test_effort)
            robot.apply_action(action)
            sim.step()

        q_after = robot.data.joint_pos.cpu()
        tau_after = robot.data.applied_torque.cpu()

        print("Joint position after excitation:")
        print(q_after[:, joint_idx]) # 只打印受影响关节的位置

        print("\nApplied torque (target joint):")
        print(tau_after[:, joint_idx])

        print(
            "\nCheck:\n"
            f"- Joint '{target_joint_name}' should move significantly\n"
            "- Torque should be non-zero for that joint\n"
        )
        
    except ValueError:
        print(f"❌ Joint '{target_joint_name}' not found in robot.joint_names!")
        print("Available joints:", robot.joint_names)

# ------------------------------------------------------------
# 9. Done
# ------------------------------------------------------------
print("\n==============================")
print(" Sanity Check Finished ")
print("==============================\n")

simulation_app.close()
