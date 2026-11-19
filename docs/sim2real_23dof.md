# Unitree G1 23DoF Parkour 真机部署

## 退出相机服务

```bash
python3 /home/luohaoyu/unitree_sdk2/scripts/g1_camera_status.py --stop-xr
```

当前 `deploy_real/deploy_real.py` 已适配仿真的 23DoF Parkour。
默认只读：订阅真实关节/IMU/遥控器、采集深度并推理，不创建 LowCmd 发布者，
不释放机器人内置控制。原 29DoF 入口保存在 `deploy_real/deploy_real_29dof.py`。

## 内部 IMU 朝向纠偏

当前实机配置默认 `path_control.enabled: true`、`mode: heading`、`reference: current`。
直接使用 LowState 中已有的 pelvis IMU 四元数保持朝向，无需外部定位文件。
开始直行时记录当前朝向；右摇杆转向优先，松开后继续前进时更新参考；停止时输出零速度指令，再次前进时重新记录朝向。
纠偏增益为 2.0，偏航指令限幅为 0.5 rad/s，前进速度上限仍为 0.3 m/s。

这只纠正朝向误差，不能测量或消除朝向不变时的纯横向漂移。未知 XY 在诊断中记为 `null`。
要恢复完全手动的速度指令，将 `path_control.enabled` 改为 `false`。
纠偏开关不会创建电机发布者，默认只读以及 `--enable`、Start/A 的接管要求保持不变。
71 项离线回归测试通过，包含实际推理线程的指令历史检查；本次未进行实机运动验证。参数和可选定位接口见 [path_following.md](path_following.md)。

## 本机实测状态

本次出现“前进不迈步、恢复超时退出”的进一步分析见 [policy_no_walk_20261113.md](policy_no_walk_20261113.md)。限幅恢复不等于平衡恢复；现已补充退出期间的状态记录 `shutdown.feedback`，以区分故障前和阻尼退出后的机身失稳。原有阻尼退出不提供站立保持。

### 关节跟踪误差的分级恢复

当前 `control.tracking_recovery.enabled: true`。策略阶段不再因一次超过 0.8 rad 的跟踪误差立即退出，而是尝试有时限的限幅恢复：

1. 常规限位、限速后，若目标与实测角之差超过 0.8 rad，进入恢复状态，仍持续执行策略推理和 5 ms 电机循环，不使用 sleep 等待关节。
2. 发送目标取三个范围的交集：关节绝对位置范围、上一发送目标 ± 本周期允许变化量、实测位置 ±0.8 rad。正常关节不额外改动，不增大 kp/kd，不把关节目标突然清零。
3. 本次出现过异常的关节，其限位后策略需求误差和限速后候选误差都降到 0.65 rad 以内，并由连续新状态维持 0.1 s 后，结束恢复。不能用已经截断的发送值判断“已恢复”，也不能因为误差在 0.8 附近抖动就重启计时。
4. 一次恢复最长 0.5 s。超时、限速后候选误差超过 1.0 rad，或上述约束没有交集时，仍拒绝发送并走原有故障阻尼退出。没有交集时不会强行跳变目标来追随实测位置。

```yaml
control:
  max_position_error_rad: 0.8
  tracking_recovery:
    enabled: true
    hard_error_rad: 1.0
    release_error_rad: 0.65
    timeout_s: 0.5
    release_hold_s: 0.1
```

这些是初始工程参数，不是经过实机验证的恢复保证。1.0 rad 是候选目标的立即故障阈值，不是允许发送 1.0 rad 跟踪误差；恢复期间发送值仍受 0.8 rad 约束。
只有 `POLICY` 阶段允许恢复，姿态爬升、稳定确认和 Ready 阶段保持严格保护。状态/动作/深度超时、电机故障、温度、姿态超限和遥控停止仍正常生效。
策略原始动作历史不改写，原始动作与最终执行目标的差异仍需闭环验证。
将 `tracking_recovery.enabled` 改为 `false` 可以恢复“第一次超限立即退出”的旧行为。

终端输出恢复开始/结束提示，会话报告和最近电机记录包含 `tracking_recovery`：恢复次数、成功结束次数、限幅周期数、异常关节索引以及超时/约束冲突原因。
两次历史左踝故障的单步回放均可限幅继续，但不能据此推断真实机器人后续会恢复。
合成 MuJoCo 策略起步对照中，严格模式在 0.765 s 退出；恢复模式成功结束 8 次恢复后，在 8.580 s 因跟踪误差与限速约束无交集退出。该测试没有完成 20 s 行走，不是稳定性通过结果。
原有落地等待 Ready 的合成仿真还会在策略接管前因姿态倾斜退出，未绕过实机 Ready 检查来掩盖该问题。
完整原始结果见 `logs/sim2real/tracking_recovery/`。本次未连接机器人、未发送真实电机指令。

离线闭环复现命令（`--policy-start` 仅用于隔离策略阶段的合成仿真，不代表实际接管流程）：

```bash
.venv-parkour/bin/python scripts/validate_real_handover.py \
  --seconds 20 --flat --vx 0.24 --wz -0.46 --policy-start \
  --tracking-recovery on --report /tmp/recovery_on.json
.venv-parkour/bin/python scripts/validate_real_handover.py \
  --seconds 20 --flat --vx 0.24 --wz -0.46 --policy-start \
  --tracking-recovery off --report /tmp/recovery_off.json
```

### 既有只读检查

2026-11-12 配置适配及只读检查结果：

- 本机 `enp49s0` 为 `192.168.123.222/24`，运动电脑 `.161` 和相机电脑 `.164` 均可达。
- 真实 LowState 订阅 3 秒通过，CRC 正常，`mode_machine=4`，23 个有效电机故障位均为 0。
- 读取的 quaternion/gyro/关节位置/速度/温度通过离线数值及姿态检查。
- 两个 ONNX 模型校验和通过，200 步合成输入推理通过。
- 37 项回归测试通过，包括只读模式、稀疏电机命令序列化、CRC/tick、相机协议、
  状态保护及模拟 Start→Ready→A 转换；其中启用输出的测试使用 mock 传输，不连接真机。
- 64 帧非零合成姿态/关节/速度/深度历史（含 reset）与仿真适配器对照通过。
- 经用户授权，临时停止机载 `teleimager.image_server`（PID 2982），未修改自启动。
  真实深度采集通过：5 秒收到 243 个不同帧，最大帧龄 66.66 ms，
  策略裁剪区域最小有效像素比例 99.48%。报告：`logs/sim2real/camera_5s.json`。
- 真实 LowState + 真实深度连续只读联调 30 秒通过：1501 次策略推理，平均 50.03 Hz。
  平均推理耗时 3.65 ms，最大 21.50 ms；推理结束时最大状态帧龄 25.01 ms、
  最大深度帧龄 110.91 ms，均低于当前 watchdog 阈值。
  单次耗时可能超过 20 ms，这个结果不是硬实时保证。
  报告：`logs/sim2real/monitor_30s.json`。
- 本次没有创建 LowCmd 发布者、释放机器人内置控制或发送电机指令，
  真实电机输出和步态仍未验证。
- 测试进程已退出并释放相机，最终占用检查未见持有者。
  XR 服务当前保持临时停止状态，便于继续 Sim2Real；其自启动配置未变。
  占用记录：`logs/sim2real/camera_ownership_after.json`。

## 2026-11-13 相机连接故障

此前通过的采集记录不代表当前连接状态。最新检查中 D435i 报告 USB 2.1，
不支持配置的 848×480@60 Hz；三个已列出的兼容模式也均无帧。
需要先检查机载 D435i 的 USB 数据链路，详见
[相机 USB 故障记录](camera_usb_failure_20261113.md)。
代码现会明确报告不支持的模式，初始化失败也会保存 FAIL 报告。

## 启动命令

```bash
source .venv-parkour/bin/activate
# 离线：不连接机器人或相机
python deploy_real/deploy_real.py --self-test --steps 200

# 只订阅真实状态，不打开相机，不发送 LowCmd
python deploy_real/deploy_real.py --state-only --seconds 5 --report logs/sim2real/lowstate.json

# 仅测试真实相机，不使用 DDS
python deploy_real/deploy_real.py --camera-check --seconds 5 --report logs/sim2real/camera_5s.json

# 真实状态与深度只读联调，默认模式
python deploy_real/deploy_real.py --seconds 10 --report logs/sim2real/monitor.json
```

不加 `--seconds` 可持续只读运行。Select/F1 或 Ctrl+C 结束。
`--network` 可覆盖配置中的网卡，`--config` 指定其他真机 YAML。

相机被占用时程序会明确报错并退出，不会自动停止已有图像服务。
先确认 XR 视频当前可以中断，再正常停止占用者；机载采集退出后会释放相机。
本机已有的占用检查工具默认只读：

```bash
python3 /home/luohaoyu/unitree_sdk2/scripts/g1_camera_status.py
```

需要恢复 XR 时，先退出本项目的相机采集/只读程序，然后在机载电脑上运行：

```bash
bash /home/unitree/start_teleimager.sh
```

恢复后 XR 会重新占用 RealSense，不能与本项目直接采集同时运行。

## 电机输出阶段

真实状态、深度检查通过，且确认机器人机型、相机安装和支撑/停止条件后，
由操作者显式运行以下命令才允许输出：

```bash
python deploy_real/deploy_real.py --enable
```

本次适配没有执行这个命令。LowState 的 mode 4 与本机原 23DoF 导出涉及不同硬件版本，
槽位兼容不意味着电机动力学完全一致；需要在支撑条件下确认参数与执行效果。

1. 保持双摇杆回中，程序进入 `waiting`，此时不发送电机指令。
2. 新按一次 **Start**：检查状态与深度，释放内置运动控制服务，随后开始有电机输出。
3. 约 3 秒目标过渡后进入 `settling`。实测关节角度、速度和机身角速度连续达标 0.5 秒后才显示 `ready`；当前最多等待 8 秒，仍未达标则保护退出。终端分别显示位置误差最大关节、速度最大关节、未达标指标及连续稳定时间。
4. 机器人达到默认关节姿态且摇杆回中后，新按一次 **A**，进入 `policy`。
   策略历史重新初始化，目标在约 1 秒内渐入。
5. 左摇杆前推给前进速度，右摇杆左右给偏航角速度；未手动转向时默认使用内部 IMU 朝向保持，不使用横向位置反馈。
6. **Select/F1** 或 Ctrl+C 退出；已接管时尝试发送约 1 秒阻尼指令再关闭连接。

连接时已按住的 Start/A 不触发状态转换；A 必须在 Ready 后重新按下。
数据失效或不可恢复的保护错误会停止策略并退出，需要查明原因后重启；策略阶段的短暂跟踪误差按上文分级恢复规则处理。
程序异常终止、网络断开或进程被强杀时软件不能保证阻尼消息送达，
独立停止手段仍需在实际机型上确认。

### 姿态稳定超时案例（2026-11-13）

报告 `logs/sim2real/real_failure_vgtvbiye.json` 在 `settling` 阶段等待 3.005 秒后退出，策略尚未启用。
左髋最大位置误差 0.1071 rad，小于 0.25 rad 阈值；机身角速度模长 0.1509 rad/s，小于 0.2 rad/s 阈值。
真正未达标的是左踝 pitch（motor 4）的角速度绝对值 0.2660 rad/s，超过 0.2 rad/s。
等待过程还记录到约 1.27 rad/s 的机身晃动，连续稳定计时被打断；旧报错只突出位置误差最大的左髋，容易误判失败关节。

现将 `control.ready_timeout_s` 从 3.0 改为 8.0，给予暂时晃动更多消退时间，但位置、关节速度、机身角速度阈值及连续 0.5 秒达标要求不放宽。
停止键和原有故障保护继续生效；等待期间仍维持默认姿态控制，不会因达到等待期限自动进入策略。
新的 `blocking_checks` 指出具体未达标指标，`fastest_joint` / `fastest_motor_slot` 单独标明速度最大的关节。
99 项离线回归测试通过，新增测试覆盖延迟稳定、始终不稳定、稳定保持时间不足及时间戳倒退。原日志在退出时结束，没有证据证明延长等待后本次物理晃动一定会消失；本次未发送实机控制指令。

## Start 后仍瘫软或姿态未到达

本次真实日志发现双踝默认姿态误差持续约 0.26 rad，旧版只按计时显示 Ready。
已增加 `settling` 实测准入和 motor mode / tau_est 记录，详见
[Ready 误报与真实状态分析](ready_posture_failure.md)。
这修正了误报，不表示真实支撑问题已解决；不要在瘫软状态继续按 A。

## 按 A 后触发关节跟踪误差

2026-11-14 已定位到与 `left_ankle_pitch_joint, elapsed=6.550s` 完全匹配的真实报告，
并结合操作者确认的持续外部支撑分析了目标、实测与力矩轨迹，见
[踝关节反复超限的原因与处理顺序](ankle_tracking_failure_20261114.md)。

已在合成的双脚承重场景中复现同类切入错误，详见
[承重切入诊断](real_handover_analysis.md)。该实验不等于真机故障重放。

`Target/measured joint tracking error exceeds limit` 表示：策略渐入、位置限幅、
目标变化率限制之后的候选目标，与某个关节实测角度的差值超过
`max_position_error_rad`，当前为 0.8 rad（约 45.8°）。该周期候选目标在发送前被拒绝；
此前已发送的目标仍可能存在跟踪误差。程序随后尝试阻尼退出。

仅凭旧版通用错误不能判断具体关节、起因或发生时间。
`policy_steps` 是整个进程的推理计数，包括按 A 之前的等待/Ready 阶段，
不能把例如 2318 步解释为按 A 后运行了 46 秒。

已补充故障诊断，不放宽阈值、不改变策略渐入、PD 或动作：

- 终端给出关节名、硬件槽位、候选目标/实测值/误差、阶段和进入该阶段后的时间。
- 默认保存 `logs/sim2real/real_failure_*.json`，无需额外 `--report`；指定报告路径时使用该路径。
- 报告记录所有关节的原始策略输出、缩放后目标、渐入目标、限位及限速后的候选目标、
  上一周期目标、实测 q/dq、IMU、摇杆、数据帧龄和最后约 5 秒的已发送命令采样。
- 保存触发时对应的 624 维本体观测与 8 帧深度输入，便于离线重放。
  报告在阻尼和清理后写入；清理异常不会覆盖原始故障或阻止后续清理。

离线检查报告，不访问机器人或相机：

```bash
python scripts/replay_real_failure.py logs/sim2real/real_failure_实际文件名.json
```

核查顺序：先确认误差发生的关节和实际 q 是否跟随；再比较原始策略目标、动作渐入、
IMU/深度输入及真实支撑状态。悬空/关节受约束、承重控制不足或目标变化异常都可能产生
跟踪误差，但需故障快照才能区分。不要以取消检查或扩大 0.8 rad 阈值代替定位。
之前只读联调通过不代表电机响应或真实步态通过。新增诊断未在本轮主动触发真机策略。

## 配置与接口

真机配置：`deploy_real/config/g1_23dof.yaml`。
策略配置仍引用 `config/g1_23dof/parkour/params/deploy.yaml`，模型不复制、不重新导出。
旧 `deploy_real/config/real.yaml` 仅服务原 29DoF 入口。

| 项目 | 当前实现 |
| --- | --- |
| 关节 | 原生 G1 23DoF，不自动锁定 29DoF 的额外电机 |
| 紧凑 23 索引 → HG 槽 | `0..12,15..19,22..26` |
| 策略顺序 → HG 槽 | 使用 metadata 的 `action_sdk_motor_indices` 并按名称交叉校验 |
| 无效槽 | 13、14、20、21、27、28 及其余保留槽均 mode=0 |
| IMU | `LowState.imu_state` pelvis 坐标，quaternion 为 wxyz，gyro 为 rad/s |
| 动作 | `offset + raw_action * scale`，按仿真相同顺序与尺度 |
| 控制 | PR 模式；`mode_machine` 与接收到的模式相符，当前配置预期为 4 |
| LowCmd | 使用正确字段 `dq`，设置 q、kp、kd，tau=0；固件执行 PD |
| 周期 | 策略 0.02 s；LowCmd 0.005 s，推理线程与发送线程分离 |
| 遥控器 | 机器人 `wireless_remote`，不使用本机 USB Xbox |
| 速度限制 | 初始真机配置 vx 0～0.3 m/s，wz ±0.5 rad/s，vy=0 |

仿真每步直接计算并限幅力矩；这里采用官方 SDK 的电机位置 PD 语义，
没有复用旧项目的“主机计算 PD、仅发送 tau”控制链。
metadata 的 effort limits 保留用于审计，但 HG 命令没有独立的运行时力矩上限字段；
实际限幅由固件执行。机型/固件、惯量和接触差异需实机验证，不能称为与仿真动力学等价。

输出目标有 metadata 位置限幅、6 rad/s 目标变化率限制和跟踪误差检查。
这些处理影响真实执行，策略历史里的 `last_action` 仍是原始网络输出。

保护配置包括状态/动作 0.1 s、深度 0.15 s 超时，倾斜、温度、关节速度、
motorstate 故障位和非有限数检查。CRC 错误、重复 tick 不刷新新鲜度；
倒退 tick 和模式变化终止控制。推理停止产出时主发送循环独立触发超时。
Python/DDS 调度并非硬实时保证。

## 深度与仿真对齐

本机既有相机配置为 SSH `unitree@192.168.123.164`，D435i SDK 串号
`233722073805`，原始 Z16 深度 848×480@60 Hz。
SSH 密钥和 known_hosts 指向既有 `~/unitree_sdk2/.g1-ssh`，没有复制或输出私钥。
其他机器需修改 `camera.identity_file`、`camera.known_hosts` 及设备参数。

相机来源支持 `ssh` 或本机 `realsense`。SSH 只运行临时相机采集 worker，
策略和电机通信留在当前电脑；退出时 worker 正常关闭相机。
采集程序本身不会停止设备占用者或修改机载自启动配置。
本次由用户明确授权，单独通过已有工具临时停止 XR 服务后完成联调。

- 根据真实内参映射到 36×64、58.29° 垂直视场的虚拟针孔相机，保持光学 Z 米。
- 非有限值/小于 0.1 m 值转 2.5 m，裁顶部 18 行和左右各 16 列。
- 高斯 3×3、sigma=1，裁剪至 0～2.5 m 并归一化。
- 策略每 20 ms 采样深度；37 帧历史、步长 5、固定 delay=0，取旧→新 8 帧。
- 预处理和历史与当前仿真一致，不启用旧真机项目的 inpaint 或随机延迟。
- 无效像素过多、数据过期、帧号倒退/重复、损坏协议均拒绝继续。
- SSH 时间戳用本地请求时间减去远端帧龄，避免不同时钟直接相减；重复读取不刷新帧龄。

虚拟视场映射不能修复真实相机安装姿态与训练安装的差异。
相机外参仍需结合实际安装核对，不能仅凭图像尺寸一致判定完成标定。

## 依赖与测试

本机 `.venv-parkour` 已增加 Unitree Python SDK 和 CycloneDDS。
Python 3.11 的 CycloneDDS 0.10.2 在本机从源码编译，安装于 `.deps/cyclonedds`，
transport 自动设置本地库路径，不修改系统库。

重建真机依赖：

```bash
bash scripts/setup_real_dependencies.sh
python -m unittest discover -s tests -v
python -m pip check
```

深度采集/SSH 传输参考本机 `~/unitree_sdk2/deploy/g1_23dof`，许可见
`deploy_real/CAMERA_UPSTREAM_LICENSE`。控制状态机为本项目实现。

保留原模式时：

```bash
source .venv/bin/activate
python deploy_real/deploy_real_29dof.py
```
