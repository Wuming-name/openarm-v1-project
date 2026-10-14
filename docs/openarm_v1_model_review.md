# 用户模型接入检查（2026-10-14）

已读取原文件 `D:\openarm_rl\models\openarm_v1_7dof.xml`，与用户粘贴内容一致。
XML 原样打包，未改变质量、坐标、限位、摩擦、接触过滤或几何。
只进行了 XML/资源存在性检查，没有加载真实 MuJoCo 引擎或运行电机。

| 关节 | 模型名称 | XML 中的范围（不是已核实的实机安全范围） |
|---|---|---|
| J1 | `openarm_right_joint1` | -80.0° ～ 200.0° |
| J2 | `openarm_right_joint2` | -10.0° ～ 190.0° |
| J3 | `openarm_right_joint3` | -90.0° ～ 90.0° |
| J4 | `openarm_right_joint4` | 0.0° ～ 140.0° |
| J5 | `openarm_right_joint5` | -90.0° ～ 90.0° |
| J6 | `openarm_right_joint6` | -45.0° ～ 45.0° |
| J7 | `openarm_right_joint7` | -90.0° ～ 90.0° |

七个 worldbody hinge、七个 motor actuator、无 freejoint、无 J8/夹爪活动 body。
仍有夹爪/左臂未使用 asset 声明，保留相应资源以确保完整加载；这些声明本身不等于动态刚体质量。
已检查 57 个 mesh 声明对应资源均存在，去重后 43 个文件。
XML SHA256: `159e28c192f3b45655da135c1273fe42352c13843aeb949ba09c77ae4e50c47e`。

建议 Ubuntu 位置：`/home/lsk/openarm_v1_ws/src/openarm-v1-project/models/openarm_v1_right/openarm_v1_7dof.xml`，`meshes/` 与 XML 同级。
工具新增 `tools/static_torque/check_model.py`（纯离线、不访问 CAN）和
`tools/static_torque/safety.model_pending.json`（只填模型路径/名称/模型限位，全部确认标志仍为 false）。
不要覆盖已有核实的 safety.local.json，也不要把 pending 配置直接当作运动许可。

需核实：

- 实物基座安装方向与模型相符。模型 right_link0 有约 +90° 绕 X 旋转，零位时 J1 轴约为世界 -Y，
  不是世界竖直轴；不能假定 J1 重力力矩应为零。
- link7 质量 0.465977 kg 是否已经排除实物拆掉的 J8 电机和夹爪。
  仅删除关节/夹爪 body 不能证明剩余 link7 惯性数据正确；不要根据未知组成直接减去某个电机质量。
- link2 质量、各 link 质心是否对应实际装配。静态重力主要受质量、质心、重力和角度影响。
- 模型 damping=0.4、frictionloss=0.1。纯静止重力对比不会自动补偿实机静摩擦。
- 模型 motor 的 40/27/7 Nm forcerange 是仿真参数，不是实机已核实工作限值。
- 有多对 collision exclude，且基座 collision 使用了视觉网格 body_link0_3.obj；
  无接触结果不能替代实物碰撞检查。
- 默认 visual 类引用未声明材料 visualgeom，但当前各实例覆盖了 material；
  是否影响你的引擎加载须由真实 MuJoCo 加载结果判断，不据此擅自改文件。
- 自检快照 J4=0.097848 rad，而原计划工具默认需要至少
  path_margin 0.02 + tracking_alarm 0.16 + startup_tolerance 0.005 = 0.185 rad，
  还未加零点不确定性。所以从这个起点按默认配置会被拒绝。
  应依据实际保持误差、标定和边界重新设计起点/裕量，不能仅为通过检查而放宽限位或填零不确定性。

先加载模型和打印快照对应的模型重力；之后再填写实机增益、保护值、零点误差与姿态计划。
