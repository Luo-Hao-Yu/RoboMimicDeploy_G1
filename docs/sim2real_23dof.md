# G1 23DoF Parkour 实机启动

启动顺序见 [README](../README_zh.md)。依次完成离线自检、LowState 检查、相机检查和只读联调，再决定是否启用电机输出。

## 网卡和相机

默认配置为 [g1_23dof.yaml](../deploy_real/config/g1_23dof.yaml)。换电脑时，更新 `network` 以及 `camera` 下的 `ssh_host`、`identity_file`、`known_hosts` 和 `serial`。可以用 `--network` 临时指定网卡，或用 `--config` 选择另一份配置。

```bash
ip -br addr show dev enp49s0
```

## 切换 XR 相机服务

如需从 XR 切换到 Parkour 深度采集，在部署电脑上运行以下命令。脚本位于同级 `~/unitree_sdk2` 项目。

```bash
python3 "$HOME/unitree_sdk2/scripts/g1_camera_status.py"
python3 "$HOME/unitree_sdk2/scripts/g1_camera_status.py" --stop-xr
```

如需切回 XR，先退出 Parkour 深度采集，再在机载相机电脑上运行：

```bash
bash /home/unitree/start_teleimager.sh
```
