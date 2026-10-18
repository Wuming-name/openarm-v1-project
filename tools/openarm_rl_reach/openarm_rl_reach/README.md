# OpenArm V1 七关节 PPO 到点

完整操作见同目录 `使用说明_训练与实机运行.txt`。

这是源代码包，不包含已经训练成功的策略；保留你现有 MuJoCo XML 和 meshes。

Ubuntu 安装位置：

`~/openarm_v1_ws/src/openarm-v1-project/tools/openarm_rl_reach`

基本顺序：

1. 独立 `.venv` 安装依赖，CMake 编译 CAN 桥。
2. `robot.py init` 创建本机配置。
3. 支撑并失能时 `capture-home`、`capture-goal` 记录经过验证的起终姿态。
4. `robot.py inspect` 检查模型和名义路径。
5. `train.py` 在仿真训练，`train.py --evaluate ...` 评估。
6. `robot.py timing ...` 带 CAN 只读查询和推理检查，不使能。
7. 检查本机配置，显式 `robot.py deploy ... --execute`，输入 `RUN_REACH` 才运动。

初版是一个记录目标的局部到点任务，输入 34 维、输出 7 维，前馈力矩为零。
不支持任意姿态自动回零，也不宣称没有外部测量时的绝对末端定位精度。
程序退出/异常只尝试失能；失能不等于机械臂不会坠落。

默认部署命令不访问 CAN 或使能；训练和评估完全离线。
`VERIFY.txt` 说明已经完成的测试和未验证部分。
