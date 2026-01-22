#!/usr/bin/env python3
"""
测试RM26 Engineering V2 Reach环境配置

该脚本用于验证环境是否正确配置，包括：
1. 环境是否能正确加载
2. 观测空间维度是否正确
3. 动作空间维度是否正确
4. 环境是否能正常运行多个步骤
"""
from isaaclab.app import AppLauncher
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app
import gymnasium as gym
import torch

# Isaac Lab imports
from isaaclab.envs import ManagerBasedRLEnv


def test_env_creation(env_name: str = "Template-Isaac-Reach-RM26-v0"):
    """测试环境创建"""
    print(f"\n{'='*60}")
    print(f"测试环境: {env_name}")
    print(f"{'='*60}\n")
    
    try:
        env = gym.make(env_name, num_envs=4, render_mode=None)
        print("✓ 环境创建成功")
        return env
    except Exception as e:
        print(f"✗ 环境创建失败: {e}")
        return None


def test_observation_space(env: ManagerBasedRLEnv):
    """测试观测空间"""
    print("\n" + "-"*60)
    print("观测空间测试")
    print("-"*60)
    
    obs, _ = env.reset()
    
    # 获取观测组
    if "policy" in obs:
        policy_obs = obs["policy"]
        print(f"✓ 观测维度: {policy_obs.shape}")
        print(f"  - 环境数: {policy_obs.shape[0]}")
        print(f"  - 观测维度: {policy_obs.shape[1]}")
        
        # 期望维度: 7(joint_pos) + 7(joint_vel) + 7(pose_command) + 7(actions) = 28
        expected_dim = 28
        if policy_obs.shape[1] == expected_dim:
            print(f"✓ 观测维度正确 (期望: {expected_dim}, 实际: {policy_obs.shape[1]})")
        else:
            print(f"✗ 观测维度不匹配 (期望: {expected_dim}, 实际: {policy_obs.shape[1]})")
            
        # 检查观测值范围
        print(f"  - 观测值范围: [{policy_obs.min():.3f}, {policy_obs.max():.3f}]")
        print(f"  - 观测值均值: {policy_obs.mean():.3f}")
        print(f"  - 观测值标准差: {policy_obs.std():.3f}")
        
        return True
    else:
        print(f"✗ 未找到'policy'观测组, 可用组: {list(obs.keys())}")
        return False


def test_action_space(env: ManagerBasedRLEnv):
    """测试动作空间"""
    print("\n" + "-"*60)
    print("动作空间测试")
    print("-"*60)
    
    action_space = env.action_space
    print(f"✓ 动作空间类型: {type(action_space)}")
    print(f"✓ 动作维度: {action_space.shape}")
    
    # 期望维度: 7 (右臂6个关节 + 1个末端关节)
    expected_dim = 7
    if action_space.shape[0] == expected_dim:
        print(f"✓ 动作维度正确 (期望: {expected_dim}, 实际: {action_space.shape[0]})")
    else:
        print(f"✗ 动作维度不匹配 (期望: {expected_dim}, 实际: {action_space.shape[0]})")
    
    print(f"  - 动作下界: {action_space.low}")
    print(f"  - 动作上界: {action_space.high}")
    
    return True


def test_env_step(env: ManagerBasedRLEnv, num_steps: int = 10):
    """测试环境步进"""
    print("\n" + "-"*60)
    print(f"环境步进测试 ({num_steps} 步)")
    print("-"*60)
    
    obs, _ = env.reset()
    
    for step in range(num_steps):
        # 随机动作
        actions = torch.rand((env.num_envs, env.action_space.shape[0]), device=env.device) * 2 - 1
        
        try:
            obs, rewards, terminated, truncated, info = env.step(actions)
            
            if step == 0:
                print(f"✓ 第 {step+1} 步执行成功")
                print(f"  - 奖励维度: {rewards.shape}")
                print(f"  - 平均奖励: {rewards.mean():.4f}")
                print(f"  - 终止数量: {terminated.sum()}")
                print(f"  - 截断数量: {truncated.sum()}")
            elif step == num_steps - 1:
                print(f"✓ 第 {step+1} 步执行成功")
                print(f"  - 平均奖励: {rewards.mean():.4f}")
                
        except Exception as e:
            print(f"✗ 第 {step+1} 步执行失败: {e}")
            return False
    
    print(f"✓ 所有 {num_steps} 步执行成功")
    return True


def test_command_generation(env: ManagerBasedRLEnv):
    """测试命令生成"""
    print("\n" + "-"*60)
    print("命令生成测试")
    print("-"*60)
    
    try:
        # 获取命令管理器
        if hasattr(env, 'command_manager'):
            commands = env.command_manager.get_command("ee_pose")
            print(f"✓ 命令维度: {commands.shape}")
            print(f"  - 环境数: {commands.shape[0]}")
            print(f"  - 命令维度: {commands.shape[1]} (3位置 + 4四元数)")
            print(f"  - 位置范围: X[{commands[:, 0].min():.3f}, {commands[:, 0].max():.3f}], "
                  f"Y[{commands[:, 1].min():.3f}, {commands[:, 1].max():.3f}], "
                  f"Z[{commands[:, 2].min():.3f}, {commands[:, 2].max():.3f}]")
            return True
        else:
            print("✗ 未找到命令管理器")
            return False
    except Exception as e:
        print(f"✗ 命令生成测试失败: {e}")
        return False


def test_reset(env: ManagerBasedRLEnv, num_resets: int = 5):
    """测试环境重置"""
    print("\n" + "-"*60)
    print(f"环境重置测试 ({num_resets} 次)")
    print("-"*60)
    
    for i in range(num_resets):
        try:
            obs, _ = env.reset()
            print(f"✓ 重置 {i+1}/{num_resets} 成功")
        except Exception as e:
            print(f"✗ 重置 {i+1}/{num_resets} 失败: {e}")
            return False
    
    print(f"✓ 所有 {num_resets} 次重置成功")
    return True


def main():
    """主测试函数"""
    print("\n" + "="*60)
    print("RM26 Engineering V2 Reach 环境配置测试")
    print("="*60)
    
    # 测试环境创建
    env = test_env_creation("Template-Isaac-Reach-RM26-v0")
    if env is None:
        print("\n✗ 测试失败: 无法创建环境")
        return
    
    # 运行所有测试
    tests = [
        ("观测空间", lambda: test_observation_space(env)),
        ("动作空间", lambda: test_action_space(env)),
        ("命令生成", lambda: test_command_generation(env)),
        ("环境步进", lambda: test_env_step(env, num_steps=10)),
        ("环境重置", lambda: test_reset(env, num_resets=5)),
    ]
    
    results = {}
    for test_name, test_func in tests:
        try:
            results[test_name] = test_func()
        except Exception as e:
            print(f"\n✗ {test_name}测试异常: {e}")
            results[test_name] = False
    
    # 汇总结果
    print("\n" + "="*60)
    print("测试结果汇总")
    print("="*60)
    
    passed = sum(results.values())
    total = len(results)
    
    for test_name, result in results.items():
        status = "✓ 通过" if result else "✗ 失败"
        print(f"{status}: {test_name}")
    
    print(f"\n总计: {passed}/{total} 测试通过")
    
    # 关闭环境
    env.close()
    
    if passed == total:
        print("\n✓ 所有测试通过！环境配置正确。")
    else:
        print(f"\n✗ {total - passed} 个测试失败，请检查配置。")


if __name__ == "__main__":
    main()
