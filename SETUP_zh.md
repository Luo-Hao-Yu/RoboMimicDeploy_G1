# 本机环境配置

## 默认 23DoF Parkour

默认入口 `deploy_mujoco/deploy_mujoco.py` 使用本机已配置好的
`.venv-parkour`，Python 为 3.11.18。VS Code 默认解释器已指向此环境。
已有终端请重新激活：

```bash
source .venv-parkour/bin/activate
python deploy_mujoco/deploy_mujoco.py
```

此入口等价于在 `/home/luohaoyu/deploy_mujoco` 中运行：

```bash
python deploy_mujoco.py --config config/g1_23dof/parkour/params/deploy.yaml
```

机器人直接由策略站立，双摇杆回中后启用控制，无需按 Start 或 LB+X。
左摇杆上推前进（0～0.65 m/s），右摇杆左右转向（±1 rad/s）。
松杆指令归零，手柄断连后保持零指令并重试连接；不启用后退或横移。
关闭 MuJoCo 窗口退出。配置为默认运行 600 秒并显示深度预览。

```bash
python deploy_mujoco/deploy_mujoco.py --simulation-duration 10
python deploy_mujoco/deploy_mujoco.py --control keyboard
python deploy_mujoco/deploy_mujoco.py --gamepad-device /dev/input/js1
```

键盘模式：小键盘 6/4 增减前进速度，9/3 增减转向速度，5 清零。
部署 YAML 位于 `config/g1_23dof/parkour/params/deploy.yaml`，
设置 `policy_interface.preview_depth: false` 可关闭深度预览。
`deploy_mujoco/config/mujoco.yaml` 是原 29DoF 的配置，不控制新的默认入口。

## Parkour 依赖与重建

为避免两个项目物理引擎版本不同带来的轨迹差异，
`.venv-parkour` 使用与源项目相同的 MuJoCo 3.6.0、ONNX Runtime 1.23.2、
NumPy 2.2.6、SciPy 1.15.3、GLFW 2.10.0 和 OpenCV 5.0.0.93。
完整运行依赖固定在 `requirements-parkour.txt`。

两个 ONNX 模型均使用 CPU 推理，GPU 用于 MuJoCo 图形和深度渲染。
Parkour ONNX 运行环境无需 PyTorch。

Python 3.11 独立发行版已从源项目复制到本项目 `.deps/python311`。
重建环境可使用该解释器，或已有的 Python 3.11：

```bash
.deps/python311/bin/python3.11 -m venv .venv-parkour
source .venv-parkour/bin/activate
python -m pip install -r requirements-parkour.txt
python -m pip check
```

机器人、木板地形、策略配置与 ONNX 已复制到本项目，不依赖源项目路径运行。
机器人网格复用已有 `g1_description/meshes`，29 个引用文件均已与来源逐一核对。
详细来源和许可见 [复用说明](deploy_mujoco/UPSTREAM.md)。

## Parkour 验证结果

```bash
source .venv-parkour/bin/activate
python -m unittest discover -s tests -v
python scripts/validate_g1_23dof.py --seconds 10 --report logs/g1_23dof/standing.json
```

离屏检查需要宿主机 GPU/图形设备访问权限；脚本默认使用 EGL。

- 16 项测试通过：关节/PD/动作映射、本体与深度历史、ONNX 哈希和接口、
  木板几何、默认入口与手柄控制。
- `pip check` 无依赖冲突。
- 与源项目分别使用各自的运行环境执行相同的 6 秒测试：
  前 2 秒零速，之后前进 0.35 m/s，第 4 秒起同时转向 0.1 rad/s。
  初始 qpos、本体观测、初始深度，以及全部 300 次策略动作与关节轨迹
  逐元素一致，最大绝对差为 0。
  对照报告位于 `logs/g1_23dof/source_comparison.json`。
- 10 秒零速站立未跌倒，未离开路线；报告位于 `logs/g1_23dof/standing.json`。
- 默认入口完成 5 秒图形窗口和深度预览运行，并识别真实 Xbox 手柄。
- 完整木板路线通行尚未验证。原项目在持续前进时也需要手动纠正转向。

## 原 29DoF / CUDA PyTorch 环境

原来的 `.venv` 仍保留 Python 3.8.10，以及 PyTorch 2.3.1、torchvision 0.18.1、
torchaudio 2.3.1 的 CUDA 12.1 版本。GPU 运算已在 RTX 3070 Ti Laptop GPU 上验证。

原 29DoF FSM 仿真使用独立入口：

```bash
source .venv/bin/activate
python deploy_mujoco/deploy_mujoco_29dof.py
```

平躺场景：

```bash
python deploy_mujoco/deploy_mujoco_29dof.py xml_path=g1_description/g1_29dof_LieDown.xml
```

原模式的操作顺序为 Start → 等待约 2 秒 → LB+X 起身 → RB+A 行走。
原模式的舞蹈等组合键仅适用于该 29DoF 入口。
其依赖固定在 `requirements.txt`、`requirements-torch-cu121.txt` 和 `requirements-lock.txt`。

原 Unitree SDK editable 安装位于 `.deps/unitree_sdk2_python`，
提交为 `814556d15970dd2ecf1c9984e845ca02ab07e206`。
真机入口 `deploy_real/deploy_real.py` 和配置仍属于原 29DoF 流程；
网卡为 `deploy_real/config/real.yaml` 中的 `enp49s0`，未进行真机通信验证。
