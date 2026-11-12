"""Run the G1 23DoF Parkour deployment copied from ~/deploy_mujoco."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

# Resolve local imports both when run as a script and with ``python -m``.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from deploy_mujoco.deploy_mujoco_env import (
    DEFAULT_CONFIG,
    DEFAULT_FSM_CONFIG,
    run_deploy,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Unitree G1 23DoF Parkour MuJoCo deployment")
    parser.add_argument("--config", type=str, default=DEFAULT_CONFIG, help="Path to deploy.yaml")
    parser.add_argument(
        "--fsm-config",
        type=str,
        default=str(DEFAULT_FSM_CONFIG),
        help="FSM config for mimic motion lookup",
    )
    parser.add_argument("--xml-path", type=str, default=None)
    parser.add_argument("--simulation-dt", type=float, default=None)
    parser.add_argument("--simulation-duration", type=float, default=None)
    parser.add_argument("--control", choices=("keyboard", "gamepad"), default=None,
                        help="Override config control mode (Parkour defaults to gamepad)")
    parser.add_argument("--gamepad-device", default=None, help="Linux joystick path, e.g. /dev/input/js0")
    parser.add_argument("--gamepad-deadzone", type=float, default=None, help="Stick deadzone in [0, 1)")
    parser.add_argument("--device", type=str, default="cpu", help="TorchScript device when exported/policy.pt is used")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    run_deploy(args.config, args)


if __name__ == "__main__":
    main()
