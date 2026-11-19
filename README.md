# RoboMimicDeploy G1

Run the commands from the repository root.

## 23DoF Parkour · MuJoCo (Sim2Sim)

### Simulation startup check

```bash
source .venv-parkour/bin/activate
python -m pip check
python scripts/validate_g1_23dof.py --seconds 10 --report logs/g1_23dof/standing.json
```

### Simulation startup

```bash
# Default gamepad mode. Center both sticks, then use the left stick to move forward
# and the right stick to turn. Close the MuJoCo window to stop.
python deploy_mujoco/deploy_mujoco.py

# Keyboard alternative. Numpad 6/4 adjusts forward speed, 9/3 turns, and 5 zeros the command.
# python deploy_mujoco/deploy_mujoco.py --control keyboard

# Select a different joystick device if needed.
# python deploy_mujoco/deploy_mujoco.py --gamepad-device /dev/input/js1
```

## 23DoF Parkour · Robot (Sim2Real)

### machine startup check

```bash
source .venv-parkour/bin/activate

# Offline ONNX check
python deploy_real/deploy_real.py --self-test --steps 200

# Check the configured Ethernet interface.
ip -br addr show dev enp49s0

# Optional when switching from XR to Parkour depth capture.
python3 "$HOME/unitree_sdk2/scripts/g1_camera_status.py" --stop-xr

# Check state and camera independently, then run the read-only policy loop.
python deploy_real/deploy_real.py --state-only --seconds 5 --report logs/sim2real/lowstate.json
python deploy_real/deploy_real.py --camera-check --seconds 5 --report logs/sim2real/camera_5s.json
python deploy_real/deploy_real.py --seconds 10 --report logs/sim2real/monitor.json
```

### machine startup

```bash
# After the checks pass and with the robot secured, allow motor output.
python deploy_real/deploy_real.py --enable

# Center both sticks. Press Start, wait for "Phase: ready", then press A.
# Left stick forward moves, right stick left/right turns.
# Press Select/F1 or Ctrl+C to stop.
```