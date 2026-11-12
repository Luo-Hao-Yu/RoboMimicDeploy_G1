# 本机环境配置

本项目使用项目目录内的 `.venv`，Python 为系统提供的 3.8.10。
VS Code 默认解释器已指向 `.venv/bin/python`；已有终端需手动激活。

在项目根目录运行：

```bash
source .venv/bin/activate
python deploy_mujoco/deploy_mujoco.py
```

需要先连接 Xbox 手柄。平躺起身场景使用：

```bash
python deploy_mujoco/deploy_mujoco.py xml_path=g1_description/g1_29dof_LieDown.xml
```

## 依赖版本

- PyTorch 2.3.1、torchvision 0.18.1、torchaudio 2.3.1，均为 CUDA 12.1 版。
- NumPy 1.24.4：替代原 README 的 1.20.0，以满足 ONNX Runtime 的依赖要求，并保持 NumPy 1.x ABI。
- MuJoCo 3.2.3、pygame 2.6.1、PyYAML 6.0.2、SciPy 1.10.1：补齐代码实际使用的依赖。
- Unitree SDK 安装在 `.deps/unitree_sdk2_python`，使用 editable 安装。
  源码提交为 `814556d15970dd2ecf1c9984e845ca02ab07e206`。
- ONNX Runtime 使用 CPU 版，与原 README 一致；安装 CUDA 版 PyTorch 不会自动将全部策略改为 GPU 推理。

直接依赖见 `requirements.txt`，完整安装版本快照见 `requirements-lock.txt`。

系统未安装 Conda，也未提供 `python3.8-venv` 的 ensurepip，因此本次通过
PyPA 的 Python 3.8 virtualenv zipapp 创建 `.venv`，未修改系统 Python 包。

## 重建环境

```bash
curl -fL https://bootstrap.pypa.io/virtualenv/3.8/virtualenv.pyz -o /tmp/robomimic-virtualenv.pyz
python3 /tmp/robomimic-virtualenv.pyz .venv
source .venv/bin/activate
python -m pip install -r requirements-torch-cu121.txt
python -m pip install -r requirements-lock.txt
mkdir -p .deps
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git .deps/unitree_sdk2_python
git -C .deps/unitree_sdk2_python checkout 814556d15970dd2ecf1c9984e845ca02ab07e206
python -m pip install -e .deps/unitree_sdk2_python
python -m pip check
```

## GPU 检查

```bash
nvidia-smi
python -c "import torch; print(torch.__version__, torch.version.cuda); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"
```

宿主机检测到 RTX 3070 Ti Laptop GPU 和 NVIDIA 570.133.20 驱动。
Codex 沙箱内可能无法访问 `/dev/nvidia*`；GPU 检查应在宿主机终端运行。

真机部署入口为 `deploy_real/deploy_real.py`，网卡参数位于
`deploy_real/config/real.yaml`，已设置为本机有线网卡 `enp49s0`。
本次配置时该接口未连接网线；真机通信需要接入机器人网络后验证。

## 已完成验证

- `pip check`：无依赖冲突。
- PyTorch CUDA 可用，GPU 矩阵乘法通过。
- 全部 11 个策略初始化与单步推理通过，输出维度正确且无非有限值。
- 默认场景及平躺起身场景均能加载，并各完成 100 步物理仿真。
- 已识别 Xbox 360 Controller（6 个轴、11 个按钮）。
- 完整仿真入口成功打开 MuJoCo 图形窗口，运行 3 秒后自动关闭。
- Unitree SDK 消息、CRC 及真机入口导入通过；未启动真机通信。
