# RM26 Reach环境配置参数详解

## 观测空间配置详解

### 为什么选择这些观测？

#### 1. 关节位置 (joint_pos_rel) - 7维
```python
joint_pos = ObservationTermCfg(
    func=general_mdp.joint_pos_rel,  # 相对于默认位置
    noise=Unoise(n_min=-0.01, n_max=0.01),  # ±0.01弧度噪声
    params={"asset_cfg": SceneEntityCfg("robot", joint_names=["right_j[0-5]", "right_end"])}
)
```

**作用**: 
- 告诉策略当前机械臂的配置
- 使用相对位置而非绝对位置，提高泛化能力
- 噪声模拟传感器误差，提高鲁棒性

**取值范围**: 典型为 [-1, 1]（归一化后）

#### 2. 关节速度 (joint_vel_rel) - 7维
```python
joint_vel = ObservationTermCfg(
    func=general_mdp.joint_vel_rel,
    noise=Unoise(n_min=-0.01, n_max=0.01),
    params={"asset_cfg": SceneEntityCfg("robot", joint_names=["right_j[0-5]", "right_end"])}
)
```

**作用**:
- 提供动力学信息
- 帮助策略预测未来状态
- 用于速度相关的奖励计算

**取值范围**: 典型为 [-2, 2] rad/s

#### 3. 目标位姿命令 (pose_command) - 7维
```python
pose_command = ObservationTermCfg(
    func=general_mdp.generated_commands,
    params={"command_name": "ee_pose"}
)
```

**作用**:
- 告诉策略目标在哪里
- 包含位置 (3维) 和姿态 (4维四元数)
- 策略学习最小化当前状态与目标的差距

**组成**:
- [0:3]: 目标位置 (x, y, z)，单位：米
- [3:7]: 目标姿态 (qw, qx, qy, qz)，四元数

#### 4. 上一步动作 (last_action) - 7维
```python
actions = ObservationTermCfg(
    func=general_mdp.last_action
)
```

**作用**:
- 提供历史信息，帮助策略理解系统动力学
- 避免动作突变，保持平滑性
- 相当于给策略提供"记忆"

**取值范围**: [-1, 1]（动作空间范围）

---

## 动作空间配置详解

```python
self.actions.arm_action = general_mdp.JointPositionActionCfg(
    asset_name="robot",
    joint_names=["right_j[0-5]", "right_end"],
    scale=0.5,
    use_default_offset=True,
)
```

### 参数说明

#### scale=0.5
**含义**: 动作缩放系数
**效果**: 
- 策略输出 [-1, 1] → 实际位置增量 [-0.5, 0.5]
- 较小的scale使运动更平滑，但可能降低响应速度
- 较大的scale可以快速运动，但容易震荡

**调整建议**:
- 初期训练: 0.3-0.5（稳定为主）
- 后期训练: 0.5-0.8（提高效率）

#### use_default_offset=True
**含义**: 使用默认关节位置作为参考基准
**效果**: 
- 动作是相对于默认位置的增量
- 帮助策略在合理的关节范围内工作
- 避免关节超限

---

## 奖励函数详解

### 1. 位置跟踪误差 (L2范数) - 权重: -0.5

```python
position_tracking = RewTerm(
    func=mdp.position_command_error,
    weight=-0.5,
)
```

**计算公式**: $r = -0.5 \times \|p_{ee} - p_{goal}\|_2$

**作用**: 
- 惩罚末端与目标的欧氏距离
- 全局导航信号，引导机械臂接近目标
- 权重较大，确保位置是主要优化目标

**数值示例**:
- 距离0.5m → 奖励 -0.25
- 距离0.1m → 奖励 -0.05
- 距离0.01m → 奖励 -0.005

### 2. 位置跟踪奖励 (tanh核) - 权重: 1.0

```python
position_tracking_fine_grained = RewTerm(
    func=mdp.position_command_error_tanh,
    weight=1.0,
    params={"std": 0.1}
)
```

**计算公式**: $r = 1.0 \times \left(1 - \tanh\left(\frac{\|p_{ee} - p_{goal}\|_2}{0.1}\right)\right)$

**作用**:
- 在接近目标时给予高奖励
- tanh函数在0附近变化剧烈，远处变化平缓
- 鼓励精细控制

**数值示例**:
- 距离0.5m → 奖励 0.04
- 距离0.1m → 奖励 0.42
- 距离0.01m → 奖励 0.90

**std参数的影响**:
- std=0.1: 在10cm内快速增长
- std=0.05: 在5cm内快速增长（更严格）
- std=0.2: 在20cm内快速增长（更宽松）

### 3. 姿态跟踪误差 - 权重: -0.2

```python
orientation_tracking = RewTerm(
    func=mdp.orientation_command_error,
    weight=-0.2,
)
```

**计算公式**: $r = -0.2 \times \text{quat\_error}(q_{ee}, q_{goal})$

**作用**:
- 惩罚末端姿态与目标姿态的差异
- 使用四元数误差，范围 [0, π]
- 权重较小，优先保证位置精度

**数值示例**:
- 姿态差30° (0.52 rad) → 奖励 -0.104
- 姿态差10° (0.17 rad) → 奖励 -0.034

### 4. 动作变化率 - 权重: -0.001

```python
action_rate = RewTerm(
    func=general_mdp.action_rate_l2,
    weight=-0.001,
)
```

**计算公式**: $r = -0.001 \times \|a_t - a_{t-1}\|_2^2$

**作用**:
- 惩罚动作的剧烈变化
- 促进平滑轨迹
- 避免高频抖动

**数值示例**:
- 动作变化0.1 → 奖励 -0.00001
- 动作变化1.0 → 奖励 -0.001

### 5. 关节速度 - 权重: -0.0005

```python
joint_vel = RewTerm(
    func=general_mdp.joint_vel_l2,
    weight=-0.0005,
)
```

**计算公式**: $r = -0.0005 \times \|\dot{q}\|_2^2$

**作用**:
- 惩罚过快的关节运动
- 节约能量
- 提高安全性

---

## 命令生成配置详解

```python
self.commands.ee_pose.ranges.pos_x = (0.3, 0.7)
self.commands.ee_pose.ranges.pos_y = (-0.3, 0.3)
self.commands.ee_pose.ranges.pos_z = (0.2, 0.6)
```

### 如何确定目标范围？

#### 1. 理论分析
- 计算机械臂的运动学工作空间
- 考虑关节限位
- 避免奇异构型

#### 2. 实验验证
```python
# 使用IK求解器测试
from isaaclab.utils.math import quat_from_euler_xyz
import numpy as np

# 生成测试点
test_points = []
for x in np.linspace(0.3, 0.7, 5):
    for y in np.linspace(-0.3, 0.3, 5):
        for z in np.linspace(0.2, 0.6, 5):
            test_points.append([x, y, z])

# 检查IK可解性
reachable = []
for point in test_points:
    if ik_solver.solve(point) is not None:
        reachable.append(point)

print(f"可达率: {len(reachable)/len(test_points)*100:.1f}%")
```

#### 3. 调整策略
- **初期训练**: 缩小范围，确保100%可达
- **中期训练**: 扩大到80%可达区域
- **后期训练**: 包含一些边界情况

### 姿态范围说明

```python
self.commands.ee_pose.ranges.roll = (-math.pi / 4, math.pi / 4)
self.commands.ee_pose.ranges.pitch = (0.0, 0.0)
self.commands.ee_pose.ranges.yaw = (-math.pi, math.pi)
```

- **Roll**: ±45°，允许适度侧倾
- **Pitch**: 0°，保持水平（根据任务需求）
- **Yaw**: ±180°，全方位旋转

---

## PD控制器参数详解

```python
"right_arm": ImplicitActuatorCfg(
    joint_names_expr=["right_j[0-5]", "right_end"],
    effort_limit=87.0,
    velocity_limit=2.0,
    stiffness=400.0,
    damping=80.0,
)
```

### 参数调优指南

#### Stiffness (刚度)
**物理意义**: 位置跟踪的"弹簧系数"

**效果**:
- **高刚度** (>500): 快速响应，但易震荡
- **中刚度** (200-500): 平衡性能
- **低刚度** (<200): 响应慢，但平滑

**调整建议**:
```
if 震荡:
    stiffness *= 0.7
if 响应慢:
    stiffness *= 1.5
```

#### Damping (阻尼)
**物理意义**: "减震器"，消耗能量

**效果**:
- **高阻尼** (>100): 过阻尼，运动缓慢
- **临界阻尼** (stiffness/5): 最优
- **低阻尼** (<stiffness/10): 欠阻尼，震荡

**调整建议**:
```
optimal_damping = 2 * sqrt(stiffness * inertia)
# 对于典型机械臂: damping ≈ stiffness / 5
```

#### Effort Limit (力矩限制)
**物理意义**: 最大输出力矩

**调整建议**:
- 根据电机规格设定
- 留20%余量保证安全
- 过高可能损坏硬件

#### Velocity Limit (速度限制)
**物理意义**: 最大关节速度

**调整建议**:
- 根据电机规格和减速比
- 安全速度 = 最大速度 × 0.7
- 影响到达速度，不影响精度

---

## 训练超参数建议

### Episode长度

```python
self.episode_length_s = 10.0
```

**考虑因素**:
- **任务难度**: 简单任务可以短一些 (5-8秒)
- **探索需求**: 需要多步规划的任务要长 (10-20秒)
- **计算效率**: 短episode可以更快收集经验

**经验法则**:
```
episode_length ≈ 3 × 预期完成时间
```

### 控制频率

```python
self.decimation = 2  # 60Hz → 30Hz
self.sim.dt = 1.0 / 60.0
```

**权衡**:
- **高频率** (50-100Hz): 精确控制，但计算量大
- **低频率** (10-20Hz): 计算快，但控制精度降低
- **推荐**: 20-30Hz 适合大多数机械臂任务

### 并行环境数

```python
self.scene.num_envs = 2048
```

**考虑因素**:
- **GPU内存**: 更多环境需要更多显存
- **采样效率**: 更多环境 = 更快学习
- **推荐配置**:
  - RTX 3090 (24GB): 2048-4096
  - RTX 4090 (24GB): 4096-8192
  - A100 (40GB): 8192-16384

---

## 调试技巧

### 1. 可视化调试

```python
# 启用命令可视化
self.commands.ee_pose.debug_vis = True

# 添加末端轨迹可视化
# (需要在环境中添加marker)
```

### 2. 奖励分析

```python
# 在训练脚本中记录各项奖励
import wandb

wandb.log({
    "reward/position": position_reward,
    "reward/orientation": orientation_reward,
    "reward/action_rate": action_rate_reward,
    "reward/total": total_reward,
})
```

### 3. 动作幅度检查

```python
# 检查动作分布
print(f"动作均值: {actions.mean(dim=0)}")
print(f"动作标准差: {actions.std(dim=0)}")
print(f"动作范围: [{actions.min()}, {actions.max()}]")
```

### 4. 成功率统计

```python
# 定义成功标准
success_threshold = 0.05  # 5cm
distances = torch.norm(ee_pos - target_pos, dim=-1)
success_rate = (distances < success_threshold).float().mean()
```

---

## 常见问题排查

### Q: 训练不收敛
**可能原因和解决方案**:
1. 目标不可达 → 缩小目标范围
2. 奖励设计问题 → 增大position_tracking_fine_grained权重
3. 观测维度错误 → 运行test_rm26_reach_env.py检查
4. PD参数不当 → 降低stiffness，增大damping

### Q: 机械臂震荡
**解决方案**:
1. 降低stiffness: 400 → 200
2. 增大damping: 80 → 150
3. 减小action scale: 0.5 → 0.3
4. 增大action_rate惩罚: 0.001 → 0.01

### Q: 动作不平滑
**解决方案**:
1. 增大action_rate权重
2. 添加历史动作的平滑滤波
3. 使用EMA动作: `action_t = 0.9 * action_t-1 + 0.1 * new_action`

### Q: 学习速度慢
**解决方案**:
1. 增加并行环境数
2. 调整PPO超参数 (learning rate, batch size)
3. 使用课程学习
4. 预训练或模仿学习

---

## 参考资料

1. **Isaac Lab文档**: https://isaac-sim.github.io/IsaacLab/
2. **RSL-RL代码**: https://github.com/leggedrobotics/rsl_rl
3. **强化学习教程**: OpenAI Spinning Up
4. **PD控制理论**: Modern Robotics by Kevin Lynch

---

最后更新: 2026-01-17
