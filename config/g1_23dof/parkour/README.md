# Unitree G1 23DoF Parkour

在项目根目录运行：

```bash
source .venv-parkour/bin/activate
python deploy_mujoco/deploy_mujoco.py
```

也可显式指定同一配置：

```bash
python deploy_mujoco/deploy_mujoco.py --config config/g1_23dof/parkour/params/deploy.yaml
```

启动后直接执行策略，双摇杆回中后启用手柄。左摇杆上推前进（0～0.65 m/s），
右摇杆左右转向（±1 rad/s）。横移、后退及竖直方向命令不启用。
旧版 Start / 起身 / 舞蹈组合键不适用于此入口。

机器人和木板地形位于 `g1_description/g1_23dof_parkour`，
两个 ONNX 模型和校验元数据位于 `exported`。
部署 YAML、ONNX、机器人和地形文件均从用户指定的源项目原样复制。
启动时校验模型 SHA256、关节顺序、动作参数及本体观测接口。

设置 `params/deploy.yaml` 中的 `policy_interface.preview_depth: false`
可关闭深度预览。ONNX 使用 CPU 推理，MuJoCo 使用 GPU 渲染。

运行测试及离屏站立检查：

```bash
python -m unittest discover -s tests -v
python scripts/validate_g1_23dof.py --seconds 10 --report logs/g1_23dof/standing.json
```

机器人关节数为 23，本体历史观测 624 维，深度输入为 `(1,8,18,32)`，
编码结果为 128 维，actor 输入 752 维，输出 23 维。
物理步长 0.005 秒，策略周期 0.02 秒，decimation 为 4。

地形为 16 块木板组成的路线，与源项目相同。
站立与短时行走测试不代表已验证完整木板路线通行。
