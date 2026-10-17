OpenArm V1：J2受限位置参考修正，2026-10-17
控制器版本：openarm-static-v1-multi12-j2-trim

适用范围
针对当前实测J2稳定在约4.98度目标偏差之外的情况。本版只修正J2，不调整其他关节增益。
修正改变了运动历史；不能把本次方向分组直接视为纯正反向摩擦辨识。
所有实验目标保持原值，日志cmd是实际发送参考，goal是实验目标。MuJoCo仍按实际角度计算。

算法
开启--trim-j2后，每次到达采样目标先等待所有关节低速且位置波动满足原稳定要求。
其他6关节必须在原采样误差0.08rad范围内。J2不合格时，修正其参考值，其他参考不动。
每次参考修正为目标误差的50%，单步最大0.004rad(0.229度)。
参考相对实验目标最大偏移0.10rad(5.73度)，最多40次更新，约90秒修正预算。
每次修正采用五次平滑曲线，参考速度<=0.005rad/s，加速度<=0.02rad/s²。
只在稳定后更新，运动期间不积分。力矩预测、反馈力矩、位置跟踪、温度、通信检查仍执行。
J2实际误差<=0.003rad(0.172度)且稳定后冻结参考值，重新等待连续稳定窗口并采3秒数据。
采样期间参考不再修正；离开容差则沿用原有窗口丢弃与超时机制，不自动重新调节。
采样完成后平滑撤回J2参考补偿，再进入原计划下一段，不保留跨姿态的累积修正。
模型规划额外检查每个目标J2正负0.10rad参考区间。模型检查仍是名义采样检查，不是实际碰撞保证。
新增trim_records记录各姿态修正次数、实验目标、参考、冻结/撤回状态；失败时保留当前记录。
没有提升KP/KD、电机力矩报警、速度报警或采样容差。不保证能克服任意卡滞或消除摩擦。

安装：压缩包放到/home/lsk，Ubuntu执行
cd ~/openarm_v1_ws/src/openarm-v1-project/tools/proximal_torque
tar --exclude='./.venv' --exclude='./build' -czf "$HOME/proximal_before_trim_$(date +%Y%m%d_%H%M%S).tar.gz" .
python3 -m zipfile -e "$HOME/openarm_proximal_j2_trim.zip" .

先验证完整计划（不使能）：
.venv/bin/python -u run_proximal.py --plan-only --trim-j2 --torque-profile proximal_5_3 --approach-deg 12 --min-j4-span-deg 40 --min-j3-model-span-nm 0.3

然后短测：
.venv/bin/python -u run_proximal.py --execute --short-test --trim-j2 --torque-profile proximal_5_3 --approach-deg 12 --min-j4-span-deg 40 --min-j3-model-span-nm 0.3
短测是新计划前两个基础姿态，不保证复现旧第8姿态，也不验证整个新增角度范围。
初次上线改动控制逻辑，先检查短测J2冻结误差与参考偏移，再考虑完整运行。
输入RUN_STATIC后才使能；结束可靠支撑后输入SUPPORTED。故障为尽力失能，需确认实际状态。

完整测试：使用同一命令删除--short-test，其他参数不变。
每次都会编译/核对新控制器版本、读取当前角度和重新规划，不直接复用旧计划。
修正会显著增加时长，原规划的分钟数不含修正和撤回；每次可额外增加约90+38秒上限附近的时间。
任一故障停止，不会自动增加修正范围、力矩或重试。

结果
samples.csv：cmd1~7是实际发送参考；goal1~7是本阶段实验目标；trim_wait/trim_ramp不算有效采样。
metadata.json：trim_records记录J2修正过程；原有各完成标记和采样窗口保留。
comparison的验收以plan中的原始目标为准，J2误差上限0.003rad，其他关节仍0.08rad。
directional_metrics.csv和directional_report.json标记reference_trim_enabled。
其他关节双向位置可能仍不匹配，不宣称完成全臂精确到位或摩擦补偿。

验证范围
Python规划/数据验收/调用链离线测试；抽取实际C++修正函数，用Windows C++编译器运行简化模拟测试。
模拟覆盖固定负载偏差收敛、参考速度与偏移界限、卡滞停止、反馈无效超时、禁止修改其他轴。
模拟不是实机验证；本机无Linux SocketCAN编译环境，完整程序由Ubuntu入口编译。
不要用旧build可执行文件绕过新版入口的版本检查。
