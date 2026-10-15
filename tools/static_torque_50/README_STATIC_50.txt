OpenArm V1 单右臂 J1-J7：50 个不同静止姿态采集
版本 openarm-static-v1-multi4-static50

【先说明旧程序参数】
用户提供的旧程序与此前新版的 KP/KD 是相同的：
KP = [35,35,20,20,6,6,6]
KD = [4,4,2,2,0.8,0.8,0.8]
此前失败不能解释成“新版把 KP 调小了”。
旧程序最大跟踪误差 0.16 rad，而此前小幅诊断版是 0.03 rad。
旧程序运动速度 0.04/0.05 rad/s，此前小幅诊断版是 0.005 rad/s。
旧扫描 HALF_SPAN=0.08 rad=4.58°，不等于已经验证过全关节范围安全。

本 50 姿态模式明确恢复以下旧参数，不需要手动改配置：
  KP/KD：上述旧值
  控制周期：50 ms
  目标转移速度上限：0.05 rad/s（2.86°/s），五次平滑轨迹
  实际速度报警：0.5 rad/s
  位置跟踪误差报警：0.16 rad（9.17°）
  稳定采样位置误差上限：0.04 rad（2.29°）
  稳定采样速度上限：0.04 rad/s
  稳定窗口位置波动上限：0.003 rad
  启动位置变化容差：0.01 rad
  名义路径限位余量：0.04 rad，另加启动容差
  稳定确认：1 秒；每姿态采集：3 秒，约 60 行七关节同步数据

没有照搬旧程序的 0.9*TMAX 力矩报警：
TMAX=[54,54,28,28,10,10,10] 是协议解码范围，不能当作机械安全额定值。
当前保留反馈/预测力矩报警 [2,2,1.5,1.5,0.7,0.7,0.7] Nm 及温度 55℃ 报警。
这些是软件停止阈值，不是硬件电流/力矩硬限制；本程序不写电机内部限流参数。
此前重力/KP 近似预测不作为此模式的持位精度门槛；保留模型重力负载余量检查，
并以真实采样的角度误差、速度和波动判断是否稳定。
如果这些力矩报警阻止了运动，需要核对实机负载与额定参数，不要自动放大报警值。

【目录】
建议新建，与原工具并列，保留旧版：
/home/lsk/openarm_v1_ws/src/openarm-v1-project/tools/static_torque_50/
把压缩包内文件直接解压到这里，不要多套一层目录。
模型继续使用：
/home/lsk/openarm_v1_ws/src/openarm-v1-project/models/openarm_v1_right/openarm_v1_7dof.xml
网格文件保持模型原有的相对目录结构。
此目录的 COLCON_IGNORE 防止工具被误当成 ROS 包构建。

【首次安装】
以下都在 Ubuntu 终端，普通用户 lsk 执行：
mkdir -p ~/openarm_v1_ws/src/openarm-v1-project/tools/static_torque_50
python3 -m zipfile -e "$HOME/openarm_static_50_pose.zip" "$HOME/openarm_v1_ws/src/openarm-v1-project/tools/static_torque_50"
cd ~/openarm_v1_ws/src/openarm-v1-project/tools/static_torque_50
sudo apt install build-essential cmake nlohmann-json3-dev python3-venv
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt

【每次开机运行】
关闭其他 CAN 查询/monitor/控制程序，初始机械臂失能、有稳定支撑：
cd ~/openarm_v1_ws/src/openarm-v1-project/tools/static_torque_50
.venv/bin/python -u run_static_50.py --setup-can --execute

自动完成编译、can0 1M/5M FD 配置、七电机参数和状态读取、规划、执行及对比。
仍须在提示后输入 RUN_STATIC。运动开始前取消测量时的承重接触，保留全路径防坠。
正常结束返回起点并保持，安装支撑后输入 SUPPORTED 才会尝试失能退出。
整个程序不要 sudo 运行；只有 CAN 配置调用 sudo。
故障/中断会尝试失能，未确认电机实际失能，无硬件安全停机保证；检查真实状态。
无自动故障撤退、无自动重试。

【只生成本次 50 个姿态，不使能】
.venv/bin/python -u run_static_50.py --setup-can --plan-only
读取当前角度并保存 poses_50.csv 后退出，不发送使能/运动命令。
以后 --execute 会重新读取位置、生成新计划，不沿用过期位置。

【幅度和 50 个姿态的含义】
默认每个关节目标相对本次起点不超过正负 20°，这是候选范围上限，
不是要求每个关节都必须走满 40°；实际接受范围由模型限位、接触及负载筛选决定。
50 个目标包含单关节变化候选和多关节组合候选，采用固定随机种子生成。
每对目标至少有一个关节相差 5°；生成时检查目标、所有连接路径及各姿态回起点路径。
模型采样检查不是连续碰撞安全证明；模型零点、质量和接触过滤仍有未验证项。
此模式的限位余量沿用旧程序的名义路径约定，未将全部 0.16rad 跟踪报警叠加为几何余量。
实际角度仍逐周期检查硬限位，但无法据此证明实际臂身与环境不碰撞。
仅在已确认实体运动空间及防坠覆盖时输入 RUN_STATIC。

想改变候选范围时，支持 --max-angle-deg 10 / 15 / 20 / 30，例如：
.venv/bin/python -u run_static_50.py --setup-can --plan-only --max-angle-deg 30
30° 是更大的名义范围，旧程序只验证过小范围，不能从旧程序推断该范围实机安全。
不要通过修改模型、关闭接触检查或增加力矩阈值来强行凑够 50 个。
筛选不足 50 个，程序明确拒绝，不生成可执行计划。

稳定采样后，控制器按反馈平均角度验证：
  与此前各已采姿态至少有一个关节相差 1°；
  相对起点的最大关节变化达到该姿态请求最大变化的至少一半。
稳定采样自身还需满足每个关节目标误差 <=0.04rad。
不满足就停止，不把“不同目标但实际上基本没动”计为 50 个成功姿态。
这是电机反馈角度的区别，零位精度/输出轴标度尚未通过独立测量确认。

【保存的数据】
默认目录 ~/experiments/torque_compare/session_日期_时间_唯一编号/
  session.json                    启动步骤与结果位置
  diagnose_*/                     本次读取的参数、反馈及快照
  prepared/poses_50.csv            50 个目标：弧度和相对起点的角度变化
  prepared/coverage.json           名义角度覆盖、筛选排除原因、预计耗时
  prepared/config_50.json          本次参数
  prepared/plan.json               轨迹计划
  prepared/model.mjb               本次冻结的 MuJoCo 模型
  static_*/samples.csv             七关节角度、速度、估算力矩、温度、目标、CAN 时序
  static_*/metadata.json           状态、实测姿态检查、具体故障帧（如触发）
  static_*/comparison/poses_actual.csv   每姿态一行：7 角度(°)、7 实测力矩均值、7 模型重力力矩均值、7 残差
  static_*/comparison/summary.csv        每姿态每关节的均值、标准差、RMSE等
  static_*/comparison/report.json        是否完成全部 50 个不同姿态

比较使用每行实际反馈角度计算模型重力力矩，不使用目标角度替代实际角度。
只有正常完成采集后才自动生成 comparison；故障留下原始数据和原因，不自动冒充完整数据集。
kp_sent/kd_sent 是程序请求参数，包含协议量化误差，并非独立读取的电机内部增益。
电机反馈力矩是估算量，输出轴 Nm 标度尚未独立核准；残差包括摩擦、零位、质量和换算误差。

【主要程序】
run_static_50.py       50 姿态入口，显式选择旧参数模式；默认只诊断，--execute 才准备运动。
run_static_test.py     统一编排：编译、CAN 配置、读取、规划、采集、分析。
prepare_static_50.py   离线筛选 50 姿态与路径，不访问 CAN。
openarm_static_torque.cpp 实时 CAN 读取、平滑运动、稳定采集、保护、故障帧记录。
torque_model.py        MuJoCo 校验、数据对比、故障离线查看。
prepare_local_pilot.py/prepare_multi_pose.py 为旧模式依赖，不需要单独运行。

【验证范围】
49 项 Python 离线测试通过，含规划覆盖、路径中间碰撞拒绝、实际均值去重、
步骤失败阻止后续使能、无自动重试，以及原有对比算法回归。
本地未编译 Linux SocketCAN C++，未用真实 MuJoCo/实机验证这 50 个姿态。
Ubuntu 启动时会先自动编译并检查版本，再连接电机；具体 50 个姿态以 Ubuntu 上
本次模型筛选结果为准。旧程序能运动不等于此大范围计划已完成实机验证。
