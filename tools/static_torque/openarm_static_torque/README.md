# OpenArm V1 七关节静止姿态力矩采集

适用机器：`lsk` 的 Ubuntu 22.04 台式机；单右臂 J1–J7，已移除 J8 和夹爪。
CAN：`can0`，CAN-FD 仲裁 1 Mbps / 数据 5 Mbps；不安装或替换 CAN 驱动。
J1/J2：DM-J8009P-2EC；J3：DM-J4340P-2EC；J4：DM-J4340-2EC；J5–J7：DM-J4310-2EC V1.1。

## 当前交付状态

程序已生成，默认只做查询诊断。**未连接实机验证，当前 Windows 环境没有 Linux C++ 编译器，C++ 编译需要在台式机完成。**
Python 离线工具进行了语法和模拟数据测试；模拟数据测试不能证明真实 MuJoCo 模型或电机工作正常。

你选择先做静止姿态比较。本包不实现原程序的动态双向扫描。
模型文件未提供，机械限位、保持增益和力矩保护值还未核实，因此配置模板留空，不能直接启动运动。
这个限制来自缺少实际模型/设备数据，不是需要重新批准创建文件。

## 旧程序审查

值得保留：七关节不同协议量程、精确回复 ID 校验、内核接收时间戳、接收队列丢包检查、
超时退出、控制循环间隔检查，以及使能前重新读取姿态。

需要修改：

1. `kcan0` 改成实际 `can0`，并允许通过参数指定接口。
2. 原来的 `PERIOD_S=0.050` 是 50 ms（20 Hz），不是 20 ms。新版静态采集保留 50 ms；不宣称硬实时。
3. 原来围绕参考角 ±0.08 rad（约 ±4.6°）扫描，最终取同一个参考角的过零力矩，
   不是“不同位置下的力矩曲线”。新版改为姿态列表，所有姿态均记录七个关节。
4. 原来机械限位、KP/KD写死。原程序里 J4 的下限是 0 rad，
   启动还要求至少 0.04+0.08=0.12 rad 裕量；你曾读取 J4 约 0.098 rad，可能会被拒绝。
   **不应通过放宽下限来绕过；先核实结构/模型限位。** 新版不沿用这些未核实的运动参数。
5. 原来正反向数据存在摩擦/运动项，平均不能保证等于重力；
   不同关节回复时间不同，用一个关节的插值比例插值全部关节也不是严格同步。
   静止采集保留各关节接收时间、时间质量，离线逐行使用实际七关节角度。
6. 原程序的 `reference_gap` 没有参与最终有效性判定。新版不再使用这套过零判定。
7. 原程序仅用协议量程的 90% 检查力矩，没有独立的经核实力矩/温度报警值。
   新版配置单独的力矩、速度、温度报警，并在发送前检查估算的 PD 力矩。
   这些都是软件检查，**不是电机固件硬限流，也不能保证瞬态力矩不超限**。
8. 原程序只保存扫描阶段原始数据，诊断主要等析构时写盘。新版保存启动、移动、稳定、采样、
   返回和结束保持阶段；每行刷新，并另外保存逐事务实时诊断文件。刷新不等于断电持久化保证。
9. 原程序退出即失能，未承托的机械臂可能下落。新版正常结束回到启动姿态并继续保持，
   等输入 `SUPPORTED` 后才退出并尝试失能。Ctrl+C/故障仍会尝试失能：
   **开始运动前必须有覆盖所有姿态的防跌落保护，正常测量时保护不能承受连杆载荷。**
   USB/CAN/操作系统失效时程序无法保证安全停止，必须依靠机械保护、急停和电机固件保护。

## 文件放置

把整个文件夹放到：

```text
/home/lsk/openarm_v1_ws/src/openarm-v1-project/tools/static_torque/
    openarm_static_torque.cpp
    CMakeLists.txt
    torque_model.py
    safety.template.json
    poses.template.csv
    README.md
    COLCON_IGNORE
    requirements.txt
```

程序、配置模板、操作记录可以提交 Git。此工具单独编译，不需启动 ROS，
`COLCON_IGNORE` 使它不参与 ROS 工作空间构建。
原始实验输出统一放在 `/home/lsk/experiments/torque_compare/`，不放在项目 Git 仓库。
MuJoCo 模型可放到项目的 `models/openarm_v1_right/`，应保留 XML 引用的全部文件和目录结构。

## 第一步：在 Ubuntu 本机编译和只读诊断

先在 Ubuntu 创建目标目录，然后通过 KDE Connect / WinSCP 复制本包文件。

```bash
mkdir -p /home/lsk/openarm_v1_ws/src/openarm-v1-project/tools/static_torque
mkdir -p /home/lsk/experiments/torque_compare
```

复制完后，在 **Ubuntu 终端** 执行：

```bash
cd /home/lsk/openarm_v1_ws/src/openarm-v1-project/tools/static_torque
sudo apt update
sudo apt install build-essential cmake nlohmann-json3-dev python3-venv
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
./build/openarm_static_torque --help
```

机械臂正确接线/供电，承托好并保持失能状态；关闭其他控制器、`monitor`、参数查询程序。
只在没有其他控制程序运行时配置接口：

```bash
ip link show can0
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 sample-point 0.75 \
    dbitrate 5000000 dsample-point 0.75 dsjw 2 fd on restart-ms 0
sudo ip link set can0 up
ip -details -statistics link show can0
./build/openarm_static_torque --diagnose --interface can0
ip -details -statistics link show can0
```

诊断大约 10 秒，读取实际寄存器校验 ID、MIT 模式、波特率码和解码量程，随后查询七关节状态。
**没有任何使能、失能、位置指令或参数写入。** 诊断遇到已经使能的电机会拒绝运行。
示例输出目录为 `.../diagnose_<时间编号>/`；记下里面的 `snapshot.json` 路径。
这证明本次基本查询，不证明控制、硬实时或长期稳定性。

## 第二步：独立 MuJoCo 环境

不使用 `sudo pip`，不改变系统 Python 或 ROS 的 Python 环境。

```bash
cd /home/lsk/openarm_v1_ws/src/openarm-v1-project/tools/static_torque
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -c 'import mujoco; print(mujoco.__version__)'
.venv/bin/python -m pip freeze > /home/lsk/experiments/torque_compare/python_environment.txt
```

本包使用 MuJoCo 3.3.7 固定版本，不要求“最新版”。如果你的模型需要另一版本，
先统一修改 requirements 并验证模型加载；规划与比较必须使用同一版本。
MuJoCo 官方 Python 包自带引擎，不需要另外安装完整引擎。
来源：[官方 PyPI 包](https://pypi.org/project/mujoco/3.3.7/)。

## 第三步：填写配置和姿态（需要你的实际模型）

```bash
cd /home/lsk/openarm_v1_ws/src/openarm-v1-project/tools/static_torque
cp -n safety.template.json safety.local.json
cp -n poses.template.csv poses.local.csv
```

用 VS Code 编辑这两个 local 文件。

- `model_xml`：Ubuntu 上完整的模型 XML 路径；必须是固定基座、仅七个转动关节的右臂模型。
  不允许含 J8/夹爪自由度；删除夹爪电机的质量、保留真实仍在末端的法兰/附件质量。
  检查基座安装方向、质量、质心、惯性和碰撞几何。若有固定的其他装置，也必须核实其影响。
- `joint_names`：模型里 J1–J7 对应的七个真实 joint name；不能猜名称或按文件排列顺序默认对应。
- `joint_sign`：你表示方向相同，因此模板是全部 +1。
- `zero_offset_rad`：`q_model = sign*q_motor + offset`。0 只是暂定值，不是精确标定的证明。
- `zero_uncertainty_rad`：通过已知机械参考姿态/量测估计各零点误差；不能未知就填 0。
  这里只计算局部一阶误差敏感性，不能独立辨识或自动矫正真实零点。
- `q_min_rad/q_max_rad`：核实过的实际行程边界（电机反馈坐标），不要直接照搬旧程序或仅依赖模型。
- `kp/kd`：七关节分别核实可保持负载的增益。旧程序 35/35/20/20/6/6/6 只是历史设置，
  不是本包推荐值。新版仍是位置 PD、零速度目标、零前馈力矩；重力负载可能造成稳态位置偏差。
  数据比较使用实测角度。保持不住时停止检查，不能随意提高增益。
- `torque_alarm_nm`：经核实的保守工作报警值；54/28/10 Nm 是协议量程，不能直接作为安全工作值。
- `speed_alarm_rad_s`：实测速度报警，必须大于转移目标速度，上限 0.5 rad/s。
- `temperature_alarm_c` 等：模板有保守起始检查值，仍需按实际电机/环境确认。
- `feedback_is_joint_nm_confirmed`：确定固件反馈力矩是相应输出轴的 Nm 后设 true。
  否则仍可生成探索性对比，但不得作为定量校准结论。
- 三个 `*_reviewed`/`fall_protection_confirmed` 标志必须在实际核查后设 true；不能只改布尔值绕过准备。

`poses.local.csv` 一行一个七关节目标，单位 rad，表头必须保持原样：

```text
pose_id,q1_rad,q2_rad,q3_rad,q4_rad,q5_rad,q6_rad,q7_rad
```

建议首次只选 **2–3 个靠近已核实起始姿态的安全位置**，每个姿态同时采七关节，
之后逐步扩大范围。单关节变化时其他关节目标保持不变；测 J2/J3 不同抬臂角度通常比 J1 竖直轴
更能观察重力变化（仍取决于你的实际安装方向）。不要直接采用 ±90°/±180° 全范围扫描。
重复实验可用不同 pose_id 重复同一姿态；可从不同方向缓慢接近，比较静摩擦与重复性。

## 第四步：只在模型中检查运动路径

替换 `--snapshot` 后面的路径为第一步实际生成的文件，`--out` 目录每次使用一个新名字。

```bash
cd /home/lsk/openarm_v1_ws/src/openarm-v1-project/tools/static_torque
.venv/bin/python torque_model.py prepare \
  --config safety.local.json \
  --snapshot /home/lsk/experiments/torque_compare/diagnose_实际编号/snapshot.json \
  --poses poses.local.csv \
  --out /home/lsk/experiments/torque_compare/plan_first
```

这一步不访问 CAN。检查七关节映射、起点→各姿态→回起点的名义路径、关节裕量、模型接触、
静止被动力和重力力矩余量。路径按最多 0.005 rad 分段取样，不是连续碰撞保证。
角度裕量包含设定的零点不确定性、最大跟踪误差、启动差值；不等于真实碰撞空间裕量。
未建模的电线、桌面和其他障碍以及过滤掉的碰撞，软件无法判断，必须现场检查。
模型重力预测也可能不准，不能据此免除真实负载/电机保护核查。
计划保存完整编译模型 `model.mjb` 和 SHA256，避免之后模型变更影响本次比较。

## 第五步：执行已核实的静止姿态采集

仅在上一步通过且现场检查、保护完善后执行。起点必须与生成计划的实测起点一致；
如果移动了机械臂，应重新诊断、生成计划，不要放宽 startup_tolerance 绕过。

```bash
cd /home/lsk/openarm_v1_ws/src/openarm-v1-project/tools/static_torque
./build/openarm_static_torque --interface can0 \
  --plan /home/lsk/experiments/torque_compare/plan_first/plan.json --execute
```

输入 `RUN_STATIC` 后，程序先在失能状态预装当前姿态的 MIT 指令，并核查仍失能/姿态未变，
再发送使能指令，减小沿用历史目标的风险；此过程仍需实机验证固件行为。
七关节保持起始实测姿态，然后按五次插值缓慢转移，
等待约 1 秒连续静止稳定，采样约 3 秒，再去下一姿态。
轨迹按 50 ms 步进，不追赶延误造成的大步跳变；通信和循环超时中止。
指定的加速度是名义轨迹参数，不是实测加速度限制/保证。
每个姿态必须至少有 40 个有效采样，否则中止。
结束后回起点保持，**承托好后**输入 `SUPPORTED`；程序退出时尝试失能，不宣称已确认失能。

## 第六步：离线比较

将路径替换为第五步输出的 `static_...` 目录：

```bash
cd /home/lsk/openarm_v1_ws/src/openarm-v1-project/tools/static_torque
.venv/bin/python torque_model.py compare \
  --run /home/lsk/experiments/torque_compare/static_实际编号
```

不访问 CAN。逐行将实测七关节角度映射到模型，设置速度为 0，
取 `qfrc_bias` 的静止重力项（科氏项此时为 0）；转换回电机坐标进行比较。
这不是 `ctrl` 或 actuator_force，也不是包含摩擦/动态惯性/外接触的完整逆动力学。
参考：[MuJoCo 数据字段定义](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html)。
含接触/静止被动力的样本被排除；程序不静默把它们解释为自由空间重力。

生成文件：

| 文件 | 用途 |
|---|---|
| `samples.csv` | 全过程七关节实测/指令角度、速度、估计力矩、温度、状态、接收时间 |
| `timing.csv`、`timing.csv.jsonl` | 事务耗时、接收质量、重试及失败诊断 |
| `snapshot.json` | 起始实测姿态和寄存器配置 |
| `metadata.json`、`plan.json`、`model.mjb` | 配置、模型快照、版本、完成/中止信息 |
| `comparison/comparison.csv` | 每个有效采样、每个关节的实测力矩、模型重力及差值 |
| `comparison/summary.csv` | 每姿态/关节的均值、标准差、平均差值、RMSE |
| `comparison/report.json` | 完整性、模型 hash、零点和力矩单位的解释限制 |

分析遇到未完成实验、漏姿态或不足样本，仍可保存部分结果，但 `dataset_complete=false`，
退出码为 1；没有自动的“机械臂验证 PASS”。
同一目录第二次比较会拒绝覆盖已有结果；需要保留旧结果并使用新实验目录或手动改名比较目录。

## 下一步需要的资料

先提供 MuJoCo XML 和引用资源（可打包），或至少提供 Ubuntu 模型路径及七个 joint name；
再核实实际行程、增益和保护值。这些完成后才能把当前空姿态表填成真正可运行的多位置实验。
不要用有外力支撑时或失能时的电机力矩，直接与自由空间重力比较。
