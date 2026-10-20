# OpenArm V1 自动准备起点并运行现有策略

完整中文操作见 `操作说明.txt`。

仅支持接近零位或接近A的静止停放状态；不需要手动精确摆到A。
默认命令不使能。`home-test` / `trial` 必须显式加 `--execute` 并输入终端口令。

顺序：`inspect` → `shadow` → `home-test` → `trial`。

自动起点使用普通MIT控制；实际到位、停稳并保持后才接入RL。
当前权重没有学会停稳，因此RL单次限时5～15秒，随后外部流程减速保持。
不会更改原模型、训练配置、权重或电机零点。
需要防坠和有人值守；故障或退出可导致移除力矩。

This is an experimental, attended hardware tool. Model-only tests do not
establish hardware behavior or real end-effector accuracy.
