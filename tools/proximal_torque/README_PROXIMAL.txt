OpenArm V1 右臂近端静态力矩：分阶段持位与返回修正
版本 openarm-static-v1-multi11-return-holdfix

本版针对的问题
在base_002_inc_r1_approach，J2已经静止，却与目标保持约5.10度偏差。
原预接近容差0.08rad（4.58度）使连续稳定窗口一直无法开始。
保留前版预接近以及计划中的经起点中转容差0.10rad（5.73度），新增正常返回容差0.08rad（4.58度）。
正式采样容差仍0.08rad；初始持位仍0.04rad；正常返回持位改为0.08rad。
不是删除位置检查，不是提高电机刚度，也不是提高力矩阈值。
本版处理已确认的预接近超时和返回起点位置误差超时；不能保证所有正式采样姿态都满足0.08rad要求。
若其他阶段仍失败，保持停止，不自动放宽或重试。

保留的保护与数据要求
KP=[35,35,20,20,6,6,6]；KD=[4,4,2,2,.8,.8,.8]。
tracking_alarm=0.16rad；目标速度0.05rad/s；加速度0.08rad/s²。
proximal_5_3软件力矩报警=[5,5,3,3,.7,.7,.7]Nm，仍需显式指定。
默认legacy_low仍为[2,2,1.5,1.5,.7,.7,.7]Nm。
温度报警55℃；静态速度<=0.04rad/s；稳定窗口位置波动<=0.003rad。
先稳定1秒，再连续采样3秒；故障和失效窗口不算有效样本。
目标及路径仍检查名义模型的限位、接触、重力负载余量；没有跳过检查。
七轴仍需逐轴核验实际接近方向且位移至少1度，实测基础姿态仍需有区别。
默认12度接近偏移，比预接近0.10rad+采样0.08rad+实际方向最小1度之和更大。
本版不接受10度接近偏移，需使用12度或15度；没有自动增大偏移。
实际采样用每条记录的实际七轴角度计算模型，不能用目标角度代替。
名义模型检查并不保证实体净空；零点、质量、输出轴Nm标度仍未核准。
软件力矩报警不是电机内部电流限制或可靠硬件力矩钳制。

新增两姿态短测
--short-test：先按全实验要求筛选50个基础姿态，再只执行前2个基础姿态。
每个姿态正反各一次，默认共4次采集；幅度沿用候选计划，并不一定是小角度。
短测不强制两姿态的J1/J2跨度达到30度，不会被计为完成50姿态实验。
其余路径、方向、实际姿态区别和运动保护仍保留。
plan/config、metadata及比较报告含validation_only=true。
比较报告中的dataset_complete=true只表示该短测数据完整；
full_proximal_coverage_complete始终为false，不能当作全范围实验完成。
不带--short-test的完整实验仍要求50个基础姿态和两肩实际跨度各至少30度。

安装更新
将openarm_proximal_returnfix.zip复制到Ubuntu的/home/lsk目录。
在Ubuntu终端执行：
cd ~/openarm_v1_ws/src/openarm-v1-project/tools/proximal_torque
tar --exclude='./.venv' --exclude='./build' -czf "$HOME/proximal_before_returnfix_$(date +%Y%m%d_%H%M%S).tar.gz" .
python3 -m zipfile -e "$HOME/openarm_proximal_returnfix.zip" .
保留原.venv、build、模型和实验数据；不需要重新安装依赖。
统一入口会自动编译并检查新二进制版本，不能只替换单个Python或C++文件。
本包没有替换任何机械臂模型文件，仍使用Ubuntu工程已有模型。

短测只规划，不使能
.venv/bin/python -u run_proximal.py --plan-only --short-test --torque-profile proximal_5_3
需要重新配置can0时再加--setup-can；接口须是1M/5M CAN-FD。

执行两姿态短测
.venv/bin/python -u run_proximal.py --execute --short-test --torque-profile proximal_5_3
关闭其他CAN控制和查询工具，初始电机失能并有支撑，保持全程防坠覆盖。
重新读取当前位置和规划，检查新计划范围、实体净空和防坠后，输入RUN_STATIC。
测量姿态不要与承重支撑接触。结束返回起点保持，安装支撑后输入SUPPORTED。
软件只尽力发送失能，未确认真实失能；停止后需确认真实电机状态和支撑。

完整实验（短测成功并审阅后，再单独启动）
.venv/bin/python -u run_proximal.py --execute --torque-profile proximal_5_3 --max-proximal-span-deg 90 --min-proximal-span-deg 30 --approach-deg 12 --repeats 1
会重新规划，不复用短测起点或路线。不是自动从短测接着运行完整实验。
可能需要40分钟以上，以本次规划预计耗时为准。

结果与程序
run_proximal.py和run_static_test.py：统一入口、编译、读取、规划、执行和比较。
prepare_proximal_50.py：自适应肩肘候选与混合方向组合，所有路径检查和短测前缀。
openarm_static_torque.cpp：CAN控制、分阶段持位容差、稳定窗口、实测方向与覆盖核验。
torque_model.py：配置验证、按实际角度计算模型、短测与全量数据验收。
analyze_proximal_pairs.py：逐关节重排正反数据，保留短测标识。
其余prepare_*为公共依赖，不需手动分别运行。
结果仍在~/experiments/torque_compare/session_日期_时间_唯一编号/。
prepared/config_proximal.json、plan.json保存本次容差和短测标识。
static_*/samples.csv保存全过程；metadata.json保存接受窗口和故障。
static_*/comparison/包含summary.csv、poses_actual.csv、report.json及正反配对报告。

验证与限制
用已提供的失败数据回放：预接近位置可落在0.10rad内，并存在满足原速度/波动/时序条件的连续稳定窗口。
正式采样0.08rad保持，旧故障帧不会被误当作正式采样通过。
63项相关离线测试通过，包括短测计数、方向、数据完整性、参数上界和原始记录回放。
回放不能代替实机验证；新条件下实际摩擦状态及后续运动响应可能不同。
本地没有Linux编译器，C++修改未本地编译或实机运行；Ubuntu入口先编译再访问CAN。
不自动调增益、不自动增加力矩限制、不自动重试。

返回阶段更新说明
本次两姿态四次方向采集已全部通过，240组稳定七轴数据已写入原运行目录。
旧返回阶段要求0.04rad精确持位；捕获帧J6偏差约0.05074rad，约2.91度，故旧程序超时。
新返回阶段要求：位置偏差<=0.08rad、速度<=0.04rad/s、波动<=0.003rad，并持续稳定1秒。
这意味着回到起点附近，不意味着精确回到原始角度；实际返回偏差会写入metadata。
初始持位仍0.04rad，正式采样仍0.08rad，预接近仍0.10rad，跟踪故障门限仍0.16rad。
只改变返回阶段的质量容差，不提高任何电机力矩、速度、温度报警或增益。
旧日志只有最后的故障帧，未据此声称返回全过程满足新稳定窗口；需实机短测验证。

新增进度字段
acquisition_completed：要求的全部姿态和方向采集及实测验收完成。
return_completed：正常返回并通过返回阶段稳定窗口。
supported_exit_confirmed：收到了用户输入SUPPORTED；这是人工确认，不是传感器检测到支撑。
completed：正常退出流程完成。仍不代表电机已被硬件确认失能。
任一故障仍尽力发送失能并退出，不会因为采集完成就忽略返回故障。
旧版本缺少这些字段，不会追溯修改旧日志的完成状态。
新版本不会删除或重写旧运行数据，也不会自动将旧失败记录改为成功。
