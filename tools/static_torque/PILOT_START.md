# 首轮 1° 局部运动采集

用户已确认：旧程序 KP/KD 在本机成功使能并稳定保持，且防跌落保护已设置。
本版使用这些已验证增益；不再要求把未知的质量、零点或完整硬件行程误标成“已确认”。

此更新仅适用于靠近当前稳定起点的探索性采集，不授权大范围运动。
将更新包中的文件直接复制到 Ubuntu 的
`/home/lsk/openarm_v1_ws/src/openarm-v1-project/tools/static_torque/`，
覆盖同名代码文件，保留原有 `.venv/`、`build/`、模型和其他项目目录。
需覆盖 C++ 源文件并重新编译；旧二进制不支持这一局部试验范围。

## 运动设置

- KP：35、35、20、20、6、6、6；KD：4、4、2、2、0.8、0.8、0.8。
- 共 15 个采样姿态：起点；J1 正向 1°、回起点；J2 正向 1°、回起点；依次至 J7。
- 每次只有一个关节目标偏离起点，不同时改变七个目标；重力会导致实际保持角度略有偏差。
- 名义速度 ≤0.005 rad/s（约 0.29°/s），加速度参数 ≤0.01 rad/s²。
- 每姿态等待连续稳定约 1 秒后采样约 3 秒；总耗时通常约 3 分钟，取决于稳定时间。
- 跟踪偏差报警 0.03 rad（约 1.72°），比旧模板的 0.16 更严格；稳定位置误差阈值 0.025 rad。
- 软件力矩报警：J1/J2 2 Nm，J3/J4 1.5 Nm，J5–J7 0.7 Nm。
  这些是本次小范围试验的低值中止阈值，不是电机额定力矩，也不是固件硬限流。
- 反馈速度报警 0.15 rad/s，温度报警 55°C。超时、异常状态和丢包检查仍保留。
- 控制/采样周期仍是 50 ms；不宣称硬实时或精确加速度控制。

名义位置边界为起点 ±0.12 rad 与模型范围的交集，控制目标总偏移还被硬性限制为 1°。
路径检查继续考虑额外 0.02 rad 边界裕量、0.03 rad 跟踪报警及 0.003 rad 起点差值。
未知零点误差保留 null，不把它当已知零误差或保证的安全裕量。
模型质量/质心仍未经实测验证，模型碰撞过滤仍有效；实际物理边界与障碍必须由现场防护覆盖。

## Ubuntu 操作

先确认机械臂承托并失能，关闭其他 CAN 控制、monitor 和查询程序。
如果 `can0` 已开启并配置为 1M/5M FD，无需重新配置。

```bash
cd ~/openarm_v1_ws/src/openarm-v1-project/tools/static_torque
cmake --build build -j2
```

如果 build 尚未配置，先执行 `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release`。
新版 C++ 在当前 Windows 环境未编译，需要 Ubuntu 实际编译结果；遇到错误停止并提供输出。

重新获取新鲜、稳定的起点：

```bash
./build/openarm_static_torque --diagnose --interface can0
```

诊断成功后，保持姿态，不要改变机械臂位置，立即执行下列命令（同一终端）：

```bash
PILOT_DIR="$HOME/experiments/torque_compare/pilot_$(date +%Y%m%d_%H%M%S)"
.venv/bin/python prepare_local_pilot.py --snapshot latest --out "$PILOT_DIR"
```

这一步不访问 CAN。自动选择最近一个成功的 can0 诊断，要求起点记录不超过 3 分钟，
且诊断末尾角度稳定、七关节失能。检查模型路径、名义限位/接触/静止被动力/重力余量，
输出本次独立配置、姿态 CSV、冻结模型及 plan.json。
任何检查失败都不应强行改为通过，也不要随意提高保护值；把错误输出保留用于排查。

生成成功后查看终端里的计划和阈值，再执行：

```bash
./build/openarm_static_torque --interface can0 --plan "$PILOT_DIR/plan.json" --execute
```

输入 `RUN_STATIC` 才会使能并开始移动。
正常结束会回起点并继续保持，承托好后输入 `SUPPORTED`，再尝试失能。
Ctrl+C/通信/控制异常仍会尝试失能，机械保护需能承接下落，正常采样时不能承受连杆载荷。

## 数据与比较

实际采集数据保存在程序显示的 `~/experiments/torque_compare/static_.../`，
不是准备计划的 pilot_... 文件夹。各阶段都记录七关节的 q、dq、tau、温度与状态。

```bash
.venv/bin/python torque_model.py compare --run /home/lsk/experiments/torque_compare/static_实际编号
```

结果写入该实验目录的 comparison/，包含逐采样比较和按姿态汇总。
零点误差未知，所以零点误差数值界限输出为空/null；不能把残差全部归因于模型。
此轮仅确认局部运动采集与重复性，不适合据此辨识完整机械臂参数或宣称全角度准确。
看到明显抖动、保持不住或中止阈值触发时停止排查，不要直接扩大角度或提高增益。
