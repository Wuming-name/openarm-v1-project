OpenArm V1 右臂：近端大范围 5x5x2 姿态、双向接近力矩采集
版本 openarm-static-v1-multi7-proximal

一、这版解决什么
上次50姿态仅要求任意三个轴有跨度，J1/J2实际只覆盖约12.8°/11.0°。
本版强制对J1和J2分别取5级角度，配合两种肘部配置，共50个基础姿态。
优先搜索肩部总跨度90°、75°、60°、45°的可行组合，先照顾两轴中较小的跨度。
90°是候选总跨度上限，不是正负90°，也不保证实机能执行到90°。
默认要求J1、J2实际跨度各>=30°，并为采样位置偏差预留余量；不足拒绝完成。
不会用腕部运动或重复目标代替近端覆盖。

每个基础姿态从目标减去12°、目标加上12°的位置分别接近一次，七轴都这样设置。
因此默认50个基础姿态，100次方向采集；不是只有50次采样。
所有正式采样目标中J5-J7保持本次起点的目标角度，接近过程会让它们正反运动。
J3/J4提供两种经过筛选的肘部配置，扩大负载变化；本版不添加砝码。
实际角度/重力变化范围以本次coverage.json为准，不保证腕部重力一定足够大。

二、控制参数与两种明确的力矩报警配置
KP=[35,35,20,20,6,6,6]
KD=[4,4,2,2,.8,.8,.8]
控制周期50ms；目标速度上限0.05rad/s；平滑轨迹；跟踪保护0.16rad。
正式采样实际位置偏差<=0.08rad，速度<=0.04rad/s，稳定窗口波动<=0.003rad。
稳定1秒后连续采集3秒；单个保持阶段最多33秒；无自动故障重试。
起点/返回持位位置要求仍为0.04rad；预接近保持使用0.08rad。

默认 --torque-profile legacy_low：
  软件力矩报警 [2,2,1.5,1.5,.7,.7,.7] Nm，沿用上次。
另附显式选择的 --torque-profile proximal_5_3：
  软件力矩报警 [5,5,3,3,.7,.7,.7] Nm，仅提高J1-J4。
代码不会自动从低配置切换到高配置，配置名称和数值会在规划/执行前打印并保存。
温度报警55℃，角度、速度、通信检查保持。
这些是反馈及预测PD力矩的停止阈值，不是电机内部限流，也不是可靠硬件力矩钳制。
本程序不写电机内部限流参数，不改变MIT模式，不重设零点。

厂家参考资料（型号/版本与实物仍须核对）：
DM-J8009P-2EC V1.1厂家页面标称额定20Nm、峰值40Nm：
https://dmbot.cn/
DM-J4340P-2EC厂家页面标称额定9Nm、峰值27Nm：
https://www.dmbot.cn/index.php?c=show&id=101
5Nm/3Nm低于这些电机标称额定值，但不证明整臂结构、接线、供电及持续低速保持安全。
不要把额定转速工况的额定值当成静止持续保持能力；机械结构和实体工作空间另行检查。
电机反馈的输出轴Nm标度仍未独立核准，因此不能把软件报警当成精确实测载荷限制。

三、放置目录
建议与旧工具并列，不覆盖旧版：
/home/lsk/openarm_v1_ws/src/openarm-v1-project/tools/proximal_torque/
压缩包文件直接解压到这里，不要多套一层目录。
模型继续使用项目原models/openarm_v1_right/openarm_v1_7dof.xml及其网格文件。

首次安装（Ubuntu终端，普通用户）：
mkdir -p ~/openarm_v1_ws/src/openarm-v1-project/tools/proximal_torque
python3 -m zipfile -e "$HOME/openarm_proximal_torque.zip" "$HOME/openarm_v1_ws/src/openarm-v1-project/tools/proximal_torque"
cd ~/openarm_v1_ws/src/openarm-v1-project/tools/proximal_torque
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
若缺编译依赖：sudo apt install build-essential cmake nlohmann-json3-dev python3-venv

四、先只读取和规划，不使能
.venv/bin/python -u run_proximal.py --setup-can --plan-only
默认旧低报警；若要审阅5/3配置下的大范围候选：
.venv/bin/python -u run_proximal.py --setup-can --plan-only --torque-profile proximal_5_3
两条命令都不会使能或运动，但会配置CAN并读取电机，初始电机需失能且已稳定支撑。
关闭其他CAN控制和查询程序，避免它们与配置操作冲突。
查看终端给出的prepared/base_poses_50.csv和coverage.json中的绝对角度、力矩范围、预计时长。
程序先自动编译和检查版本，再访问CAN。

五、执行（下面明确选择5/3软件报警配置）
.venv/bin/python -u run_proximal.py --setup-can --execute --torque-profile proximal_5_3
若使用旧报警，省略 --torque-profile 参数即可。
启动会重新读取当前位置、重新规划，不沿用上次预览的过期位置。
在输入RUN_STATIC前，确认新显示的范围、完整路径实体净空和防坠覆盖，取消测量时承重接触。
正常结束返回起点并保持，安装支撑后输入SUPPORTED。
故障/中断会尝试失能，未确认电机真实失能；需检查支撑和真实状态。
没有自动故障撤退路径，规划中的返回路径只用于正常退出。

可选参数：
--max-proximal-span-deg 90    候选最大总跨度，可选45/60/75/90
--min-proximal-span-deg 30    最低实测跨度，可选20/30/40/45
--approach-deg 12            七轴接近偏移，可选10/12/15
--repeats 1                  每基础姿态正反各1次，可选1/2/3
默认100次采集，至少6000组七关节稳定数据；重复2/3次则12000/18000组。
大范围转移和双向预接近会明显增加时间，具体预计时长在RUN_STATIC之前显示。
不要因为目标跨度不足就自动提高报警值或关闭碰撞检查。

六、失败报告
若模型限位、接触或重力负载余量阻止完整5x5x2规划，明确失败，不使能。
prepared/planning_rejection.json记录候选范围、被拒原因与本次配置；没有可执行plan.json。
配置的编码范围PMAX/TMAX不是结构安全额定值。
当前模型只做名义路径采样检查，不是连续碰撞证明，模型接触过滤和质量/零点仍有未核准项。
程序不会因检查失败而改模型、关闭接触或增加力矩阈值。

七、采集验收
1. 50个不同基础目标，J1/J2五级x五级，两种肘部配置。
2. 每次采样后，七关节实测平均角度相对预接近实测角度的变化，须与指定方向一致且>=1°。
3. 不同基础姿态的首次实测配置，至少有一个关节相差1°。
4. J1和J2首次测量构成的实测跨度都>=请求最低跨度。
5. 每次需满足原有连续稳定窗口；作废段不会混入分析。
6. 初始实测位置需与模型规划起点一致，确认期间移动会拒绝使能。

八、结果
~/experiments/torque_compare/session_日期_时间_唯一编号/
  prepared/base_poses_50.csv   50基础姿态的绝对角度（°）
  prepared/acquisitions.csv   100次方向采样标识、方向、重复编号
  prepared/coverage.json      名义J1/J2跨度、七轴范围、模型重力范围、预计耗时
  prepared/config_proximal.json 本次配置和软件报警
  prepared/plan.json          正式目标与预接近位置
  static_*/samples.csv        全过程七轴原始反馈、目标、时序、采样段编号
  static_*/metadata.json      实际接近方向、实际跨度、完成状态、完整故障帧
  static_*/comparison/poses_actual.csv  每次采样的实测角度和实测/模型力矩
  static_*/comparison/summary.csv       每关节每次采样的均值/标准差/残差
  static_*/comparison/report.json       是否完成近端覆盖和全部方向采集
  static_*/comparison/paired_directions.csv 同一基础目标的正反方向配对结果
  static_*/comparison/paired_report.json    配对诊断汇总

正反两次完整七关节实际角度可能不同。最大差>0.5°时angles_matched=false，
不能把两次电机力矩的平均值直接当成同一姿态的重力。
residual_even=(反馈正向-各自模型重力+反馈反向-各自模型重力)/2
residual_odd =(反馈正向-各自模型重力-反馈反向+各自模型重力)/2
这两列是共同/方向相关残差诊断，不是已标定重力或摩擦系数。
载荷相关摩擦、零位、力矩换算误差及模型错误都可能影响它们。

九、文件用途
run_proximal.py             新版统一入口
prepare_proximal_50.py      5x5x2大范围肩肘规划，双向路径检查
openarm_static_torque.cpp  CAN控制、预接近、采集、实际方向和近端覆盖验收
analyze_proximal_pairs.py   同目标正反配对，实际角度匹配标记和残差分解
torque_model.py             MuJoCo检查及实际角度模型对比
run_static_test.py          公共流程编排，调用以上程序
其余prepare_*文件为依赖，不需要分别手动运行。

十、验证范围
新增14项近端规划/双向分析离线测试通过；13项启动编排和采样段隔离/比较回归通过。
使用的是解析/模拟动力学测试替身，不是用实机模型证明可执行。
本地未编译Linux SocketCAN C++，未实机测试新大范围和双向路径。
真实MuJoCo模型筛选与Linux编译在Ubuntu启动时完成；不能保证当前配置一定有可行大范围网格。
