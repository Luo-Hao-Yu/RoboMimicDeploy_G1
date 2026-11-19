# 共用路径反馈控制

G1 23 自由度机器人的 MuJoCo 仿真部署和实机部署共用 `common/path_controller.py`。
控制器在本体感知观测写入历史缓冲区、执行 encoder/actor 推理之前运行。
操作者的输入意图与最终策略指令使用独立的缓冲区。模型权重、关节顺序、动作缩放与偏置，以及深度历史帧填充方式保持不变。

## 仿真

交互式部署默认使用 `reference: current`：开始向前行走，或结束手动转向并继续直行时，采集当前 pelvis（骨盆根部）的 XY 位置和偏航角 yaw，建立新的参考直线。
持续直行期间保留该参考，并对漂移进行纠偏。因此，转身后松开右摇杆，机器人会沿新的朝向行走。
`config/g1_23dof/parkour/params/deploy.yaml` 中的 `path_control` 启用横向路径误差反馈，增益为 2、前视距离为 1 m、偏航角速度限幅为 1 rad/s。
状态真值来自 pelvis 自由关节的 XY 位置和 yaw，不使用 torso（躯干）的旋转。
在配置的速度上限内保留操作者请求的前进速度。

```bash
.venv-parkour/bin/python deploy_mujoco/deploy_mujoco.py --path-mode path
.venv-parkour/bin/python deploy_mujoco/deploy_mujoco.py --path-mode path --path-reference fixed
.venv-parkour/bin/python deploy_mujoco/deploy_mujoco.py --path-mode off
```

使用 `reference: current` 时，每次从 `STOP` 或 `MANUAL_TURN` 进入直行状态，都会采集新的参考，包括停止后再次启动。
右摇杆在现有死区范围内的噪声不会触发转向或参考重置。参考不会在每个控制帧刷新。
`manual_rebase` 是为 `reference: startup` 保留的原有选项；当前朝向参考模式会自动为每段新直行建立参考，无需该选项。
对于已知的平台中心线，使用 `reference: fixed` 和 `manual_rebase: false`，并在与位姿相同的坐标系中定义平台中心线。使用自定义 XML 时，需要核对该路径定义。
当前朝向参考模式沿操作者选择的方向行走，不保证与平台中心线对齐，也不会自动沿去程轨迹原路返回。

## 实机内部 IMU 朝向纠偏

`deploy_real/config/g1_23dof.yaml` 现在默认启用基于内部 IMU 的朝向纠偏：

```yaml
path_control:
  enabled: true
  mode: heading
  reference: current
  frame_id: lowstate_imu
  heading_gain: 2.0
  max_forward_m_s: 0.3
  max_yaw_rad_s: 0.5
  state_timeout_s: 0.15
```

朝向直接来自已经订阅的 `LowState.imu_state.quaternion`，四元数顺序为 wxyz，使用与策略观测一致的 pelvis 姿态。
无需 `/tmp/g1_pelvis_pose.json` 或任何外部定位文件，也不对原始加速度进行位置积分。
开始直行时记录当前 yaw；持续直行时按下式纠正朝向：

```text
wz = clip(2.0 * wrap(yaw_reference - yaw_current), -0.5, 0.5)
```

手动转向时优先使用右摇杆指令，松开转向并继续前进时记录新的 yaw；停止时输出严格零速度指令，重新前进时采集新的朝向参考。
前进速度保持操作者请求值并受 0.3 m/s 上限约束，vy 恒为零。IMU 四元数无效、数据过期或时间戳倒退时，现有推理故障和上层保护流程会处理故障。
日志中的 `state_source` 为 `lowstate_imu`，`heading_error_rad` 表示朝向误差；`cross_track_m` 和 `origin_xy` 为 `null`，表示没有位置测量，不能解释为横向偏差为零。

该功能抑制朝向转偏引起的跑偏，但不能检测朝向不变时的纯横向漂移，长期效果也受 IMU 航向漂移影响。
`enabled: true` 只启用指令纠偏，不自动启用电机输出；部署程序仍默认只读，控制权交接仍需原有 `--enable` 和 Start/A 流程。
要恢复手柄直接速度控制，可将 `enabled` 改为 `false`。仿真默认仍为 `path`；对照 IMU 朝向保持算法可运行：

```bash
.venv-parkour/bin/python deploy_mujoco/deploy_mujoco.py --path-mode heading --path-reference current
```

## 可选的外部定位路径纠偏

只有使用 `mode: path` 进行横向位置反馈时，才要求真实的 XY 定位。当前默认实机配置没有 `pose_source`；直接切换到 `path` 会因缺少定位源而拒绝启动。
今后有定位设备时，可同时设置 `mode: path`、匹配估计器的 `frame_id`，并添加：

```yaml
pose_source:
  type: json_file
  path: /tmp/g1_pelvis_pose.json
```

以上 `pose_source` 应放在 `path_control` 节点内。

接入定位时，估计器应以原子替换方式更新 `pose_source.path` 指定的 JSON 文件，即先写临时文件，再调用 `os.replace`。数据格式如下：

```json
{
  "monotonic_ns": 123456789000000,
  "xy_m": [1.2, -0.04],
  "yaw_rad": 0.02,
  "frame_id": "start_local",
  "valid": true,
  "confidence": 0.95
}
```

以上仅为接口示例，不是可直接使用的定位样本。不要把静态位置重复回放为实时估计。
时间戳必须表示采集时刻，并映射到部署计算机的 `time.monotonic_ns()` 时间基准，不能使用 UTC 或另一台计算机的运行时间。
XY 和 yaw 必须描述同一固定坐标系中的 pelvis 位姿，约定 +X 向前、+Y 向左、yaw 逆时针为正。
估计器负责 IMU、pelvis、相机之间的标定以及时间对齐；未经坐标变换，不要把局部 IMU 的 yaw 与世界坐标系的 XY 混用。样本更新频率至少为 50 Hz。

以下检查针对外部定位的 `path` 模式。启用前，应将坐标系 ID 配置为与估计器一致。使用 `reference: fixed` 时，还需要设置中心线原点和 yaw。
置信度过低、位置缺失或无效、位姿数据年龄超过 `state_timeout_s`，或位姿与 LowState 的时间差超过 `max_state_skew_s`，都会触发推理故障，并由现有上层控制流程处理。
程序不会默默使用过期 XY，也不会根据原始 IMU 加速度虚构位置。
如果估计器重置坐标系，应停止部署，变换或重新配置路线后再启动；时间戳倒退或坐标系 ID 变化会被拒绝。

## 操作意图与重置

同时释放前进和转向输入时，输出严格为 `[0,0,0]`。手动转向请求优先，进入 `MANUAL_TURN` 状态。
在当前朝向参考模式下，松开转向输入并保持前进输入，会以当前位姿开启新的 `PATH_FOLLOW` 直行段；如果前进输入也已释放，则等待再次请求前进时才采集参考。
固定平台路径在停止和手动转向前后始终保留原参考。
策略代次重置时，会重置实机路径控制器并清空策略历史；`DeployEnv.reset()` 会重置已挂接的仿真路径控制器。

零指令不等于经过验证的站立控制器：模型在行走后可能继续踏步。
现有实机控制权交接、看门狗和阻尼保护流程保持不变。离线验证不包含任何实机驱动操作。

可以在保存的实机会话记录中查看请求指令、最终指令及 `path_control` 诊断信息。
