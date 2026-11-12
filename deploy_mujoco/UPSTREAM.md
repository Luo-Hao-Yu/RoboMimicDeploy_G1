# 复用来源

23DoF Parkour 运行代码来自本机 `/home/luohaoyu/deploy_mujoco` 的工作区快照。
使用源项目当前文件，包含当时尚未提交的本地修改。
源项目 HEAD：`3517c3b0f03e8e9da573dbf77621102dd7be6b52`。
许可证保存在 [UPSTREAM_LICENSE](UPSTREAM_LICENSE)。

拷贝内容：

- `deploy_mujoco.py` → 当前仿真入口；增加本项目的包导入路径，默认策略改为 23DoF Parkour。
- `deploy_mujoco_env.py` → 本目录同名文件；调整项目根目录和包导入，
  将一个运行时类型别名改为 `typing.Dict` 以兼容原 Python 3.8 环境。
- `gamepad_control.py` → 本目录同名文件，未修改。
- `config/g1_23dof/parkour/{params,exported}` → 本项目同路径，逐字节复制。
- `g1_description/g1_23dof_parkour` → 本项目同路径，逐字节复制。
- `tests/test_g1_23dof.py`、`tests/test_gamepad_control.py` 和
  `scripts/validate_g1_23dof.py` → 本项目同路径；调整包导入和默认配置检查。

机器人所引用的 29 个 STL 已与本项目已有 `g1_description/meshes` 文件逐一进行
SHA256 对比，内容一致，因此直接复用本项目已有网格。
机器人上游许可证保存在 `g1_description/g1_23dof_parkour/UPSTREAM_LICENSE`。

运行不导入源项目模块，也不读取源项目资源。Python 3.11 独立发行版也已复制到
本项目 `.deps/python311`，`.venv-parkour` 使用本地解释器及独立安装的依赖。

保持源项目中的关节映射、PD 参数、动作缩放、深度预处理、历史采样、
ONNX 校验和、控制周期、手柄映射及初始姿态。
`requirements-parkour.txt` 固定了源项目实际使用的 ONNX / MuJoCo 运行依赖。
