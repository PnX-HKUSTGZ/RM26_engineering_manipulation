#!/bin/bash

# RM26 Engineering V2 Reach任务训练脚本

echo "开始训练 RM26 Reach 任务..."

export WANDB_API_KEY="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"

export WANDB_PROJECT=rm26

# 创建输出目录
mkdir -p outputs

# python scripts/rsl_rl/train.py --log_project_name rm26 \
#     --task "Template-Isaac-Reach-RM26-v0" \
#     --num_envs 256 \
#     --max_iterations 5000 \
#     --seed 42 \
#     --headless \
    
python scripts/rsl_rl/play.py --task "Template-Isaac-Reach-RM26-v0" --num_envs 2


# python scripts/rsl_rl/play.py --task Template-Isaac-Reach-RM26-Play-v0 --checkpoint model_2999.pt

    "$@"

