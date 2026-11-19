# RoboMimicDeploy G1

在仓库根目录执行以下命令。

## 23DoF Parkour · MuJoCo 仿真（Sim2Sim）

### 仿真启动检查

```bash
source .venv-parkour/bin/activate
python -m pip check
python scripts/validate_g1_23dof.py --seconds 10 --report logs/g1_23dof/standing.json
```

### 仿真启动

```bash
# 默认使用手柄。先将双摇杆回中，左摇杆上推前进，右摇杆左右转向。关闭 MuJoCo 窗口退出。
python deploy_mujoco/deploy_mujoco.py

# 键盘模式。小键盘 6/4 调整前进速度，9/3 转向，5 清零。
# python deploy_mujoco/deploy_mujoco.py --control keyboard

# 需要时指定其他手柄设备。
# python deploy_mujoco/deploy_mujoco.py --gamepad-device /dev/input/js1
```

## 23DoF Parkour · 实机（Sim2Real）

### 实机启动检查

```bash
source .venv-parkour/bin/activate

# 离线 ONNX 自检
python deploy_real/deploy_real.py --self-test --steps 200

# 检查配置中的有线网卡。
ip -br addr show dev enp49s0

# 从 XR 切换到 Parkour 深度采集时可选用。
python3 "$HOME/unitree_sdk2/scripts/g1_camera_status.py" --stop-xr

# 分别检查机器人状态和相机，再运行只读策略联调。
python deploy_real/deploy_real.py --state-only --seconds 5 --report logs/sim2real/lowstate.json
python deploy_real/deploy_real.py --camera-check --seconds 5 --report logs/sim2real/camera_5s.json
python deploy_real/deploy_real.py --seconds 10 --report logs/sim2real/monitor.json
```

### 实机启动

```bash
# 检查通过并做好机器人防坠措施后，允许电机输出。
python deploy_real/deploy_real.py --enable

# 双摇杆回中，按 Start，看到 "Phase: ready" 后再按 A。
# 左摇杆上推前进，右摇杆左右转向。
# 按 Select/F1 或 Ctrl+C 停止。
```