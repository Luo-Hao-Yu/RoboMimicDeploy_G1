# 本机运行环境

安装完成后，按 [README](README_zh.md) 启动仿真或实机部署。以下命令在仓库根目录执行。

## 23DoF Parkour

在已配置的电脑上激活现有环境：

```bash
source .venv-parkour/bin/activate
python -m pip check
```

重建环境时，优先使用本机 `.deps/python311` 中的解释器。新检出的仓库可使用系统安装的 `python3.11`。

```bash
parkour_python=.deps/python311/bin/python3.11
if [ ! -x "$parkour_python" ]; then parkour_python=python3.11; fi
"$parkour_python" -m venv .venv-parkour
source .venv-parkour/bin/activate
python -m pip install -r requirements-parkour.txt
python -m pip check
```

运行实机入口时安装 Unitree Python SDK 和 DDS：

```bash
bash scripts/setup_real_dependencies.sh
```

## 29DoF / CUDA 12.1

已有环境可直接使用 `source .venv/bin/activate`。重建时使用 Python 3.8，先安装 CUDA 版 PyTorch，再安装其余锁定依赖：

```bash
python3.8 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-torch-cu121.txt
python -m pip install -r requirements-lock.txt
python -m pip check
```

29DoF 实机还需要 Unitree Python SDK。上面的 `scripts/setup_real_dependencies.sh` 会在本仓库 `.deps/unitree_sdk2_python` 准备版本 `814556d15970dd2ecf1c9984e845ca02ab07e206`。在 29DoF 环境中安装该副本：

```bash
source .venv/bin/activate
python -m pip install -e .deps/unitree_sdk2_python
python -m pip check
```
