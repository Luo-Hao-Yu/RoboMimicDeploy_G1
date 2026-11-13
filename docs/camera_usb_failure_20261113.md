# 2026-11-13 RealSense “Couldn't resolve requests”

## 当前实测

SSH 正常，机器人 LowState 可读。失败在机载 RealSense 的 `pipeline.start(config)`。
未发现当前权限下可见的相机占用者；本轮没有停止其他进程或发布电机命令。

相同串号 `233722073805` 的 D435i 当前报告 USB 类型 **2.1**，
设备路径为 `.../usb1/1-3/...`。当前深度 Z16 模式列表中，
848×480 仅支持 6、8、10 Hz，没有项目配置的 848×480@60 Hz。
这是本次流请求无法解析的直接原因。

不能把 USB 类型变化单独归因为某条线损坏：端口、转接器、线缆、供电、
设备枚举或其他 USB 链路状态均需要现场排查。
此前成功时路径是 `.../usb2/2-3/...`，且实际采集过 848×480@60 Hz；
路径中的 usb1/usb2 是总线编号，判断速度以 SDK 的 `usb_type_descriptor` 为依据。

只读采集对照：

| 请求 | 当前 SDK 列出 | 实际采集 |
| --- | --- | --- |
| 848×480@60 | 否 | 无法解析请求 |
| 480×270@30 | 是 | 启动后等 5 秒无帧 |
| 480×270@60 | 是 | 启动后等 5 秒无帧 |
| 848×480@10 | 是 | 启动后等 5 秒无帧 |

所以只降低分辨率/帧率暂时也没有解决问题。测试后已恢复原配置，
没有把未验证的 USB2 模式设为默认，没有放宽 150 ms 深度超时。

## 处理步骤

1. 退出本项目及其他相机采集程序。
2. 检查的是 **机载电脑与 D435i 之间的 USB 连接**，不是外部电脑与 G1 的网线。
3. 检查 USB3 数据线、USB3 端口和转接设备；条件允许时重新插拔该相机 USB，
   或换已知正常的 USB3 数据线/接口，恢复枚举。避免操作机器人运动控制连接。
4. 恢复后重新执行相机检查；需要设备实际支持并能采集 848×480@60：

```bash
source .venv-parkour/bin/activate
python deploy_real/deploy_real.py --camera-check --seconds 5 --report logs/sim2real/camera_5s.json
```

相机检查通过后再进行只读联调：

```bash
python deploy_real/deploy_real.py --seconds 10 --report logs/sim2real/monitor.json
```

若 USB 恢复后仍无帧，再检查机载 USB/驱动日志或相机供电，不能用旧 PASS 报告
代替本次采集结果。

## 本次代码改进

- 本地和 SSH 采集启动前枚举指定串号的深度模式；不支持时给出 USB 类型、请求参数
  和相同分辨率下可用帧率，不自动降帧率或选另一台相机。
- SSH 将 `unsupported_profile` 独立报告，避免误判为 SSH 认证或网络故障。
- 标定消息增加实际 USB 类型和帧率。
- `--camera-check --report` 在初始化失败时也写 FAIL，避免留下旧的成功报告。
- 5 项新增测试和 10 项原深度测试通过，均不访问真实电机。

## 证据

- `logs/sim2real/camera_profiles_20261113.json`
- `logs/sim2real/camera_ownership_20261113.json`
- `logs/sim2real/camera_stream_probe_20261113.jsonl`
- `logs/sim2real/camera_profile_failure_20261113.json`
