OpenArm V1 右臂：自适应近端覆盖与双向力矩采集
版本 openarm-static-v1-multi8-adaptive-proximal

本次修正
旧规划器要求完整5×5×2网格，任一角点或路径不通过，就拒绝整个网格。
新版不再要求所有肩部位置共用两种肘部姿态。
在每个肩部轴11级候选和多个J3/J4组合中搜索，覆盖优先选50个基础配置。
这是离散候选搜索，不是全空间最优规划；搜索失败不证明不存在其他可行姿态。
没有提高任何力矩阈值、增益、速度、加速度，也没有关闭接触/限位检查。

保留的要求
50个不同基础姿态，每姿态正、反方向各采集一次，默认100次采集。
每次正式采集需60组左右连续稳定数据。作废采样段不混入分析。
J1和J2实际跨度都至少30度；目标跨度另加两端捕获误差余量约9.17度。
正式目标J5-J7仍保持起点角度，预接近过程会正反移动这些轴。
默认七轴接近偏移12度，并逐轴核验实测接近方向和幅度至少1度。
基础目标间距至少为1度实测区别要求加两端捕获余量，约10.17度。
所有目标、接近路径、转移路径和正常返回起点路径都进行名义模型检查。
路径以最大轴间隔0.015rad离散检查，不是连续碰撞或真实机械净空证明。
零点精度、模型质量和几何、输出轴力矩标度仍未核准。

路线
每个候选必须能从起点进入两种预接近位置，并能从目标返回起点。
选定50个基础目标后，按邻近顺序访问，重新检查实际串联路线。
直接转移被模型拒绝时，仅当经起点两段路径都通过，才记录via_start=true。
C++会按计划经起点转移并保持；它不会临时搜索路线或自动故障撤退。
coverage.json记录planned_baseline_transits和预计耗时，可能比旧版明显更长。
预计耗时不含额外等待稳定、额外预接近保持和结束等待安装支撑的时间。

控制与报警
KP=[35,35,20,20,6,6,6]；KD=[4,4,2,2,.8,.8,.8]。
周期50ms，目标速度上限0.05rad/s，加速度0.08rad/s²。
跟踪报警0.16rad；正式静态位置捕获偏差0.08rad。
初始/返回持位偏差仍为0.04rad；预接近和经起点中转持位为0.08rad。
静态速度<=0.04rad/s，稳定窗口波动<=0.003rad，温度报警55℃。
legacy_low：软件力矩报警[2,2,1.5,1.5,.7,.7,.7]Nm，为默认配置。
proximal_5_3：显式选择[5,5,3,3,.7,.7,.7]Nm。
模型重力筛选上限为所选报警的60%，不自动更换配置。
这些软件停止阈值不是电机内部电流限制或可靠硬件力矩钳制。
不改MIT模式、内部电流参数或零位；故障只尝试失能，未确认实际失能。

安装更新
将openarm_proximal_adaptive.zip复制到Ubuntu的/home/lsk目录。
在Ubuntu终端执行：
cd ~/openarm_v1_ws/src/openarm-v1-project/tools/proximal_torque
tar --exclude='./.venv' --exclude='./build' -czf "$HOME/proximal_torque_before_adaptive_$(date +%Y%m%d_%H%M%S).tar.gz" .
python3 -m zipfile -e "$HOME/openarm_proximal_adaptive.zip" .
保留现有.venv、build和模型，不需要重新安装依赖。启动时自动重新编译。
完整新目录首次安装才需：python3 -m venv .venv，再用.venv/bin/python -m pip install -r requirements.txt。

本次先只规划，不使能、不运动
.venv/bin/python -u run_proximal.py --plan-only --torque-profile proximal_5_3 --max-proximal-span-deg 90 --min-proximal-span-deg 30 --approach-deg 12 --repeats 1
要求can0已配置为1M/5M CAN-FD；需要重设CAN时另加--setup-can。
初始电机应失能且有稳定支撑；关闭其他控制和查询程序。
90度是候选最大总跨度，不是正负90度，也不保证选出90度覆盖。
检查prepared/base_poses_50.csv和coverage.json中的绝对角度、预计耗时和模型力矩。
若失败，prepared/planning_rejection.json记录端点可行范围、已选路径可行范围和拒绝原因。
endpoint_only_shoulder_spans_deg仅是端点范围，不能当成路径可行范围。
不会因失败而自动缩小最低跨度、提高报警或关闭接触检查。

后续执行方式（只规划结果和实体路径审阅后再使用）
相同入口将--plan-only替换为--execute，保留其余参数。
会重新读取当前位置并重新规划，执行计划可能不同；输入RUN_STATIC前再次检查。
全路径需覆盖防坠和失电情况；测量时不得有承重支撑接触。
正常结束返回起点保持，安装好支撑后输入SUPPORTED。
发生故障没有自动重试，失能只是尽力发送，需检查实际电机状态。

文件和结果
run_proximal.py：统一入口；run_static_test.py：编译、读取、规划和分析编排。
prepare_proximal_50.py：自适应规划、端点/路径筛选和路线保存。
openarm_static_torque.cpp：CAN控制、预接近和中转、稳定采样、实际覆盖验收。
torque_model.py：MuJoCo模型检查与按每个实际角度计算重力对比。
analyze_proximal_pairs.py：相同基础目标的方向配对与共同/方向相关残差。
其余prepare_*是公共依赖，不需逐个运行。
结果目录：~/experiments/torque_compare/session_日期_时间_唯一编号/
prepared/plan.json保存本次完整路线，prepared/coverage.json保存候选和规划范围。
static_*/samples.csv保存原始数据，metadata.json保存接受窗口、实际跨度和故障信息。
static_*/comparison/包含poses_actual.csv、summary.csv、report.json及paired_directions.csv。
若两方向实测七轴角度差超过0.5度，angles_matched=false，不把均值当同姿态重力。
残差分解只是诊断，不是已经辨识出的摩擦系数或标定力矩。

验证范围
新增6项自适应规划离线测试，覆盖肩肘耦合、跨度不足、路径接触、中转和路线检查。
14项近端/配对测试、13项编排测试、17项采样隔离/模型比较测试通过，共50项。
模型测试使用模拟替身，没有用你的真实MuJoCo模型证明可行。
本地未编译Linux C++或实机执行；Ubuntu启动时编译并检查二进制版本。
新姿态和经起点路径尚未经实机验证。不能保证当前模型和阈值一定允许所需覆盖。
