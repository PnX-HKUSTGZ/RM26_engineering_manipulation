#!/bin/bash

# RM26 Engineering V2 Reach任务训练脚本

echo "开始训练 RM26 Reach 任务..."

# 创建输出目录
mkdir -p outputs

python scripts/rsl_rl/train.py \
    --task "Template-Isaac-Reach-RM26-v0" \
    --num_envs 2048 \
    --max_iterations 3000 \
    --seed 42 \
    --headless \
    "$@"

