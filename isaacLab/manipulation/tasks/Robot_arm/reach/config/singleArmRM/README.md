# RM26 Engineering V2 Reach Task Environment

## 概述

这是一个针对RM26工程机械臂版本2的强化学习reach任务环境。任务目标是控制右臂末端执行器到达随机生成的目标位姿。

## 环境配置

### 机器人配置
- **机器人模型**: RM26 Engineering V2
- **控制自由度**: 右臂7个关节 (right_j0~right_j5 + right_end)
- **末端执行器**: right_end_link
- **其他部件**: 轮子、升降平台、左臂在本任务中保持静止

### 观测空间 (Observation Space)

总维度: 28维
1. **关节位置** (7维): 右臂7个关节的相对位置
2. **关节速度** (7维): 右臂7个关节的速度
3. **目标位姿** (7维): 目标位置(3) + 目标四元数(4)
4. **上一步动作** (7维): 上一时刻的动作

### 动作空间 (Action Space)

- **类型**: 关节位置控制 (Joint Position Control)
- **维度**: 7维
- **范围**: [-1, 1] (经过scale=0.5缩放)
- **控制关节**: right_j0, right_j1, right_j2, right_j3, right_j4, right_j5, right_end

### 奖励函数 (Reward Function)

| 奖励项 | 权重 | 说明 |
|--------|------|------|
| position_tracking | -0.5 | 末端位置与目标位置的L2距离（惩罚） |
| position_tracking_fine_grained | 1.0 | 使用tanh核的位置跟踪奖励（接近目标时奖励大） |
| orientation_tracking | -0.2 | 末端姿态与目标姿态的四元数误差（惩罚） |
| action_rate | -0.001 | 动作变化率的L2范数（平滑动作） |
| joint_vel | -0.0005 | 关节速度的L2范数（避免过快运动） |

### 终止条件 (Termination)

- **超时**: Episode长度为10秒（600个仿真步，300个控制步）
- **无失败条件**: 环境不会因为碰撞或其他原因提前终止

### 目标位姿范围 (Target Pose Range)

- **X轴**: 0.3m ~ 0.7m（前方工作空间）
- **Y轴**: -0.3m ~ 0.3m（左右范围）
- **Z轴**: 0.2m ~ 0.6m（高度范围）
- **Roll**: -π/4 ~ π/4
- **Pitch**: 0（保持水平）
- **Yaw**: -π ~ π（全方位旋转）
- **更新频率**: 每5秒

### 环境重置 (Reset)

- **关节位置随机化**: 相对于默认位置的0.5~1.5倍缩放
- **关节速度**: 重置为0

## 使用方法

### 训练

```bash
# 使用RSL-RL训练
python scripts/rsl_rl/train.py --task Template-Isaac-Reach-RM26-v0 --num_envs 2048

# 或使用训练脚本
./train.sh
```

### 测试/可视化

```bash
# 使用随机动作测试环境
python scripts/environments/random_agent.py --task Template-Isaac-Reach-RM26-Play-v0 --num_envs 50

# 使用训练好的策略测试
python scripts/rsl_rl/play.py --task Template-Isaac-Reach-RM26-Play-v0 --checkpoint /path/to/checkpoint
```

### 查看所有可用环境

```bash
python scripts/list_envs.py
```

## 超参数说明

### 仿真参数
- **仿真频率**: 60 Hz
- **控制频率**: 30 Hz (decimation=2)
- **Episode长度**: 10秒

### 场景参数
- **并行环境数**: 2048 (训练), 50 (测试)
- **环境间距**: 3.0m

### PD控制器参数 (right_arm)
- **刚度 (stiffness)**: 400.0
- **阻尼 (damping)**: 80.0
- **力矩限制**: 87.0 N⋅m
- **速度限制**: 2.0 rad/s

## 文件结构

```
singleArmRM/
├── __init__.py                      # 环境注册
├── rm_engineering_reach_env_cfg.py  # 环境配置
├── rsl_rl_ppo_cfg.py               # PPO算法配置
├── agents/                          # RL算法配置
└── README.md                        # 本文档
```

## 调试建议

### 1. 检查环境是否正确注册
```bash
python test_env_registration.py
```

### 2. 可视化目标位姿
环境配置中已设置 `debug_vis=True`，运行时会显示目标位姿的可视化标记。

### 3. 检查机器人URDF/USD
确保 `right_end_link` 在机器人模型中正确定义。

### 4. 调整观测噪声
在训练初期，可以降低观测噪声：
```python
noise=Unoise(n_min=-0.005, n_max=0.005)  # 减小噪声
```

### 5. 调整奖励权重
根据训练表现，可以微调奖励权重。例如：
- 增加 `position_tracking_fine_grained` 权重可以提高位置精度
- 减小 `orientation_tracking` 权重可以降低姿态约束

## 常见问题

### Q1: 机械臂震荡不稳定
**A**: 增大PD控制器的阻尼值 `damping`，或减小动作缩放 `scale`。

### Q2: 学习收敛缓慢
**A**: 
- 检查目标位姿范围是否在机械臂工作空间内
- 增大 `position_tracking_fine_grained` 奖励权重
- 减小episode长度，让机器人更频繁地尝试不同目标

### Q3: 末端执行器无法到达某些位置
**A**: 
- 检查关节位置限制
- 调整目标位姿范围
- 使用IK验证目标是否可达

## 后续改进方向

1. **增加成功终止条件**: 当末端位置误差小于阈值时提前终止episode并给予额外奖励
2. **课程学习**: 逐步增大目标位姿范围的复杂度
3. **添加障碍物**: 增加环境复杂度
4. **使用IK控制**: 尝试逆运动学控制作为baseline
5. **多任务学习**: 同时控制左右两个机械臂

## 参考

- Isaac Lab Documentation: https://isaac-sim.github.io/IsaacLab/
- RSL-RL: https://github.com/leggedrobotics/rsl_rl
