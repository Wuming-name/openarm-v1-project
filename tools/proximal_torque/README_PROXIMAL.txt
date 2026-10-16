OpenArm V1 右臂：J4覆盖与分方向力矩对比版，2026-10-16

修改内容
1. --min-j4-span-deg 40：完整50基础姿态计划中J4目标最大角与最小角之差至少40度。
   不是相对起点正负40度，不保证实测跨度达到40度。原关节限位、相对起点最大偏移不变。
2. --min-j3-model-span-nm 0.3：目标姿态的J3模型重力最大值减最小值至少0.3Nm。
   这是实验激励要求，不是电机报警值、力矩指令或校准结果。
3. 搜索同时考虑J1/J2覆盖、J4覆盖、J3重力跨度，并检查各目标及两方向预接近和路径。
   找不到满足条件的50姿态时保存planning_rejection.json并停止；不会自动降低要求或提高报警。
   搜索是有限候选集的贪心搜索，失败不代表物理上绝无可行解。
4. analyze_directional.py按每个关节真正的接近方向拆分统计，不直接用整条采集的inc/dec标签。
   同一基础姿态的重复采集合并后计算，每个方向少于3基础姿态或常量数据时r留空。
   J4实测跨度、J3在实测姿态下的模型重力跨度、双向角度偏差另存directional_report.json。
5. 已通过短测试的C++控制、采集、返回逻辑保持原样；仍显示multi11-return-holdfix。
   本版不声称改善了双向位置匹配，也不会把不匹配的双向平均值当成消摩擦结果。
   软件报警proximal_5_3仍[5,5,3,3,0.7,0.7,0.7]Nm，增益、速度、温度、位置检查不变。

安装：将openarm_proximal_j4_directional.zip传到Ubuntu /home/lsk。
cd ~/openarm_v1_ws/src/openarm-v1-project/tools/proximal_torque
tar --exclude='./.venv' --exclude='./build' -czf "$HOME/proximal_before_j4_$(date +%Y%m%d_%H%M%S).tar.gz" .
python3 -m zipfile -e "$HOME/openarm_proximal_j4_directional.zip" .
沿用已有.venv、模型及编译依赖，不删除旧数据。包内文件直接覆盖此工具目录，不覆盖整个tools。

先检查完整计划（读取诊断，但不使能）：
.venv/bin/python -u run_proximal.py --plan-only --torque-profile proximal_5_3 --approach-deg 12 --min-j4-span-deg 40 --min-j3-model-span-nm 0.3
查看打印的base_poses_50.csv与coverage.json，包含全部关节范围、J4跨度及J3模型力矩跨度。

运行完整50基础姿态、100次方向采集：
.venv/bin/python -u run_proximal.py --execute --torque-profile proximal_5_3 --approach-deg 12 --min-j4-span-deg 40 --min-j3-model-span-nm 0.3
运行入口会重新读取当前角度并重新规划，因此计划可能与上次plan-only不同。
确认RUN_STATIC前核对新计划；结束可靠支撑后输入SUPPORTED。故障不会自动重试。

可选短测试：在同一命令末尾加--short-test，仅执行新计划最前面的2基础姿态/4次采集。
短测试不保证包含J4极端位置，也不能证明完整J4或J3受力覆盖。

新结果位于每次运行comparison目录：
directional_metrics.csv：J1~J7各自正反方向的r、MAE、RMSE、偏差、样本基础姿态数。
directional_report.json：实测角度及模型力矩覆盖、配对匹配情况。
paired_directions.csv与原有对比文件保留；配对实际角度不匹配时不能做摩擦消除标定。

已有完整结果可以仅离线补充分析，不访问CAN、不运动：
.venv/bin/python analyze_directional.py --comparison-dir "$HOME/experiments/torque_compare/session_20261016_105004_1792119004893693164/static_1792119044579866056/comparison"
此命令只写directional_metrics.csv和directional_report.json，可重复更新这两个派生文件。

验证：完成离线单元测试与已有短测试CSV回放；本机未重新执行Linux C++编译，也未进行新姿态实机测试。
源文件未改动C++，运行入口仍在Ubuntu编译并核对控制器版本。
输出轴反馈力矩换算、模型质量与零点仍未确认。新方向统计是诊断，不是模型参数校准。
