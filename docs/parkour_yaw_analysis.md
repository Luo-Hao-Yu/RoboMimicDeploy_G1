# Parkour 直行右偏诊断

结论：绕过手柄后仍可稳定复现右偏，属于当前策略在此仿真中的速度跟踪偏差。
部署程序只把手柄映射为机身坐标系速度，没有航向/路线反馈，因此偏差会持续累积。
现有证据不支持将问题归因于本次代码拷贝、左右关节映射、PD 不一致或木板排列。
尚不能仅凭这些测试断言偏差具体来自网络权重、训练数据还是训练与 MuJoCo 的动力学/感知差异。

## 实验条件

- `.venv-parkour`：MuJoCo 3.6.0、ONNX Runtime 1.23.2，与来源项目版本一致。
- 原始 23DoF 机器人、ONNX、PD、动作映射和相机参数。
- 使用 EGL 离屏渲染，完全绕过手柄，前 2 秒零指令，然后指定 `vx=0.4 m/s, vy=0, wz=0`。
- 每次推理后断言进入策略的最后一帧速度指令与指定指令逐元素一致。
- 负 yaw 表示从初始 +X 朝向向右转；横向坐标以路线中心 `y=0` 为基准。
- 原木板宽度 0.9 m，机器人 pelvis 超出横向 ±0.45 m 记为离轨。
  此为路线位置判据，不是足底接触或完整路线通行判据。

## 结果

| 条件 | 第 6 秒 pelvis 航向 | 第 6 秒 y | 第 15 秒 pelvis 航向 | 第 15 秒 y |
| --- | ---: | ---: | ---: | ---: |
| 站立，vx=0 | +0.20° | +0.009 m | −0.28° | +0.009 m |
| 原配置直行，wz=0 | −20.16° | −0.170 m | −60.41° | −2.479 m |
| 临时换为平地 | −57.28° | −0.625 m | +170.50°（已绕过 −180°） | −3.222 m |
| 原地形，物理步长减半 | −18.87° | −0.162 m | −63.96° | −2.528 m |
| 临时加入航向保持 | +0.78° | +0.169 m | −0.36° | +0.787 m |
| 临时加入航向及横向位置反馈 | −5.66° | +0.063 m | −4.61° | +0.054 m |

原配置约第 7.9 秒离轨。站立最大横向偏移约 0.009 m；直行约 2.479 m；
加入航向及横向位置反馈约 0.104 m，15 秒内未跌倒或离轨。
仅保持航向时仍有侧向速度误差，约第 10.9 秒离轨。
这些反馈只存在于诊断脚本，没有启用到默认部署。

平地实验同时改变接触与深度图分布，不能据此断言木板比平地更稳定，
只能说明木板边缘和间隙不是右偏的必要条件。
步长减半仍右偏，说明这不是简单修改 decimation 就能消除的问题。

## 已排查

1. **手柄输入**：右偏在完全没有手柄参与、且实际输入 `wz=0` 时出现。
   这排除了手柄作为该复现现象的必要原因；未对用户每次现场操作做实时录制，
   因此不声称任何时候手柄都不存在额外误差。
2. **左右映射及 PD**：关节经名称映射；观测/动作顺序、缩放和偏移与导出元数据一致。
   成对电机的 kp、kd、armature 和力矩上限一致。检查的腿部碰撞几何采用一致摩擦参数。
   原配置直行中没有请求力矩超过电机 ctrlrange 的情况。
3. **模型整体朝向**：初始四元数无 yaw；站立无明显持续右旋。
   移动时 pelvis 与 torso 都偏右，不是仅腰关节摆动造成的视觉假象。
4. **迁移差异**：前次原项目/当前项目 300 个策略步动作与 qpos 的对照完全一致，
   报告在 `logs/g1_23dof/source_comparison.json`；原项目已有的直行报告也记录同方向离轨。

## 控制链路差异

当前 `deploy_mujoco/gamepad_control.py` 把右摇杆直接缩放为 `wz`，松杆为 0。
`deploy_mujoco/deploy_mujoco_env.py` 的 `_velocity_command_obs` 只裁剪指令范围；
主循环把该指令交给策略，没有用世界坐标 yaw 或横向位置计算修正量。

`wz=0` 是对策略提出“角速度为零”的要求，并不直接锁住机器人的世界朝向。
本体观测包含角速度和重力投影，而重力投影不包含绝对 yaw；
该部署也没有提供相对路线中心的显式位置误差。
深度图包含环境信息，但不等于已经存在明确的航向或路线保持控制器。

本机训练仓库当前代码中的 `PoseVelocityCommand._update_command` 会计算目标方向，
并生成 `wz = heading_error * heading_control_stiffness` 后裁剪。
所查 Parkour 配置中 stiffness=2.0，`GapCrossingCommand` 继承这套更新逻辑。
当前部署没有复用这一目标导向的外层反馈，而由操作者通过右摇杆承担转向修正。

训练代码位置：

- `/home/luohaoyu/deploy_mujoco/.tools/InstinctLab/source/instinctlab/instinctlab/tasks/parkour/mdp/commands/pose_velocity_command.py`
- `/home/luohaoyu/deploy_mujoco/.tools/training_source/source/instinctlab/instinctlab/tasks/parkour/config/parkour_env_cfg.py`

以上是本机现有训练源码，不是该 checkpoint 训练时的完整配置快照；
因此“缺少目标反馈”是已确认的代码差异及实验支持的纠偏方向，
不是对该 checkpoint 精确训练过程的断言。

## 建议

若需要推左摇杆即可沿木板中心线前进，应在速度指令层加入可配置的航向和横向位置反馈，
并明确右摇杆如何修改目标航向。只加固定 `wz` 偏置会随速度/地形变化，
仅保持 yaw 又不能消除横向速度误差。

诊断脚本的横向反馈示例采用 `desired_heading=atan2(-y,0.8)`，
`wz=clip(2*wrap(desired_heading-yaw),-1,1)`，只用于验证机制。
它读取 MuJoCo 真值，不可直接视为真机通用方案；也没有验证完整木板路线。

## 复现

```bash
source .venv-parkour/bin/activate
python scripts/diagnose_parkour_yaw.py
```

所有场景都在内存中修改，无需修改机器人文件、部署 YAML 或 ONNX。
输出包含每秒位置、yaw、实际角速度、机身速度、输入指令、力矩饱和比例及离轨时间。
默认报告路径为 `logs/g1_23dof/yaw_diagnosis.json`。

只运行原配置与反馈对照：

```bash
python scripts/diagnose_parkour_yaw.py --cases forward heading_hold cross_track --seconds 15 --vx 0.4
```
