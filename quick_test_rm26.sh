#!/bin/bash
# RM26 Engineering V2 Reach环境快速测试脚本

echo "=========================================="
echo "RM26 Reach环境配置测试"
echo "=========================================="
echo ""

# 1. 测试环境配置
echo "[步骤 1/3] 运行环境配置测试..."
python test_rm26_reach_env.py
TEST_RESULT=$?

if [ $TEST_RESULT -eq 0 ]; then
    echo "✓ 环境配置测试通过"
else
    echo "✗ 环境配置测试失败，请检查配置"
    exit 1
fi

echo ""
echo "=========================================="
echo "[步骤 2/3] 列出所有注册的环境..."
echo "=========================================="
python scripts/list_envs.py | grep -i "rm26\|engineer"

echo ""
echo "=========================================="
echo "[步骤 3/3] 运行随机动作测试 (5秒)..."
echo "=========================================="
timeout 10 python scripts/environments/random_agent.py \
    --task Template-Isaac-Reach-RM26-Play-v0 \
    --num_envs 4 \
    --disable_fabric

echo ""
echo "=========================================="
echo "测试完成！"
echo "=========================================="
echo ""
echo "下一步："
echo "1. 训练模型:"
echo "   python scripts/rsl_rl/train.py --task Template-Isaac-Reach-RM26-v0 --num_envs 2048"
echo ""
echo "2. 测试训练好的模型:"
echo "   python scripts/rsl_rl/play.py --task Template-Isaac-Reach-RM26-Play-v0 --checkpoint /path/to/checkpoint"
echo ""
