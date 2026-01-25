#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
IsaacSim 4.5：启动仿真并打开指定的 USD 模型
"""

import os
import sys

# ———————— 配置部分 ————————
USD_PATH = "/home/cytochrome/RM26_engineering_manipulation/isaacLab/manipulation/assets/usd/rm26_version2_engineering_model/rm26_version2_engineering_model.usd"
# ————————————————————————

# 检查 USD 文件路径
if not os.path.isfile(USD_PATH):
    print(f"[Error] USD 文件不存在：{USD_PATH}")
    sys.exit(1)

# =================================================================
# ⚠️ 必须在导入其他 IsaacSim/Omniverse 模块之前启动 SimulationApp
#    否则会遇到 "ModuleNotFoundError" 或其它加载错误
# =================================================================

from isaacsim.simulation_app import SimulationApp  # 启动应用
# 启动参数（可选：headless True/False，renderer 等）
simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RayTracedLighting"
})

# ----------------------------------------------------------------------------
# ⚠️ 一旦 SimulationApp 启动成功，才能导入其他 Omni/IsaacSim API 模块
# ----------------------------------------------------------------------------
import omni
import isaacsim.core.utils.stage as stage_utils

print(f"正在打开 USD Stage：{USD_PATH}")

# 使用 IsaacSim 提供的 open_stage（会替换当前 Stage）
result = stage_utils.open_stage(usd_path=USD_PATH)
if not result:
    print(f"[Error] stage_utils.open_stage 失败打开 USD")
    simulation_app.close()
    sys.exit(1)

# 等待若干帧，确保 USD stage 加载完毕
simulation_app.update()
simulation_app.update()

print("USD Scene 加载成功！你应该能在界面中看到模型。")

# ===== 主循环（显示窗口直到关闭） =====
try:
    while simulation_app.is_running():
        simulation_app.update()
finally:
    # 退出应用
    simulation_app.close()
