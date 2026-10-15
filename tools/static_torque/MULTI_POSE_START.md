# 更明显的多姿态力矩对比（第二阶段）

上一轮不是只设定一个姿态，而是 7 个关节各改变目标 1°并回起点，共 15 个采样姿态。
但它只是局部试验。原保持位置容差 0.025 rad（约 1.43°）比目标变化还大，
因此仅凭“60 stable samples saved”不能证明实测角度真的变化了 1°。

新版先读取上一轮 samples.csv 的实际角度，计算各关节相对前一基准姿态的实际变化。
上一轮实际正向变化不到 0.25°或样本不够的关节，不自动扩大运动。

## 第二阶段方案

- 默认对每关节分别评估正负 3°、5°、10°，每个方向选检查通过的最大一个角度。
- 可显式指定 --max-angle-deg 15，增加 15°候选；不是保证每个关节都能到 15°。
- 始终是单关节改变目标，其他目标保持起始值，每测一个目标都返回基准。
- 保留已验证的 KP/KD、慢速 0.005 rad/s、跟踪报警 0.03 rad，以及原有低值力矩/温度报警。
- 检查名义路径接触、限位/裕量、重力余量；另外用 |模型重力|/KP 筛除明显不能满足
  当前 0.025 rad 保持位置容差的候选。这是保守近似，实机摩擦等仍可能导致无法稳定。
- 不能通过的角度会被缩小或排除，不自动放宽阈值、不增加增益、不自动添加重力前馈。
- 采集后检查实际位移方向和幅度：至少达到所选目标偏移的 50%，否则中止，
  metadata 标记未完成，离线比较排除明确未通过实际位移检查的姿态。
- 仍基于实测七关节角度做模型比较；质量、零点和实物碰撞边界仍未校准。

大幅抬臂时需要更大保持力矩，而当前是零前馈位置 PD。
部分肩肘角度可能无法满足现有误差/力矩阈值，这时应单独分析保持控制，不能为了角度大强行放行。
检查通过的是名义模型路径，不保证真实碰撞安全；保护、现场障碍检查仍需保持。

## 文件更新

把更新包内文件直接复制到：
/home/lsk/openarm_v1_ws/src/openarm-v1-project/tools/static_torque/

覆盖同名代码文件，保留 build/、.venv/、模型、配置及旧实验数据。
新增 prepare_multi_pose.py；C++ 可执行程序需要重新编译。

## Ubuntu 操作

如上一轮仍显示 HOLDING，先承托好并输入 SUPPORTED 退出。
关闭其他控制/查询程序，确认 can0 已按 1M/5M FD 开启。

```bash
cd ~/openarm_v1_ws/src/openarm-v1-project/tools/static_torque
cmake --build build -j2
```

先报告上一轮真实角度变化（不访问 CAN）：

```bash
.venv/bin/python prepare_multi_pose.py \
  --previous-run "$HOME/experiments/torque_compare/static_1792044758710916613" \
  --report-only
```

读取新的、稳定且失能的起点：

```bash
./build/openarm_static_torque --diagnose --interface can0
```

保持起点不变，随即准备第二阶段计划：

```bash
MULTI_DIR="$HOME/experiments/torque_compare/multi_$(date +%Y%m%d_%H%M%S)"
.venv/bin/python prepare_multi_pose.py \
  --previous-run "$HOME/experiments/torque_compare/static_1792044758710916613" \
  --snapshot latest --max-angle-deg 10 --out "$MULTI_DIR"
```

此步不会控制电机。查看终端选中的实际目标角度、预计耗时，
以及 MULTI_DIR 中的 multi_poses.csv、candidate_assessment.json。
不是所有关节都必然达到 10°；若肩肘只能通过 3°/5°或某方向无候选，不要随意提高阈值。
多个大角度往返采用原慢速，可能耗时十几至几十分钟；以程序估算为准。

准备成功且现场检查后执行：

```bash
./build/openarm_static_torque --interface can0 --plan "$MULTI_DIR/plan.json" --execute
```

输入 RUN_STATIC 才会启动。正常结束承托好后输入 SUPPORTED。
遇到抖动、报警或实际位移不足，停止排查，不直接重新加大角度。

实际采集目录是终端显示的 static_...；离线比较仍使用：

```bash
.venv/bin/python torque_model.py compare --run /home/lsk/experiments/torque_compare/static_实际编号
```

## 验证范围

Python 3.10 语法和模拟数据检查完成；原 15 项检查及新增 5 项多姿态检查通过。
未在当前 Windows 环境编译 Linux C++，未执行第二阶段实机运动。
上一轮通过的编译/实机结果不等于本次新增逻辑已通过实机验证。
