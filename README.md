# GRIT Deploy Onboard G1

Onboard C++ deployment for Unitree G1 with locomotion, VR whole-body tracking,
and NPZ trajectory playback.

English | [中文](README_ZH.md)

## Acknowledgements

This project is primarily based on
[GRIT](https://github.com/mrzuang/GRIT_teleop_deploy). The whole-body control
(WBC) model
[`policy.onnx`](https://github.com/mrzuang/GRIT_teleop_deploy/blob/main/sim2real/checkpoints/policy.onnx)
is provided by GRIT.

The locomotion model
[`Unitree-G1-AMP-Flat_model_30000.onnx`](https://github.com/ccrpRepo/wbc_fsm/blob/main/model/loco/Unitree-G1-AMP-Flat_model_30000.onnx)
comes from [ccrpRepo/wbc_fsm](https://github.com/ccrpRepo/wbc_fsm).

The robot bridge and VR retargeting design refer to the `sim2real` branch of
[Axellwppr/motion_tracking](https://github.com/Axellwppr/motion_tracking/tree/sim2real).

Thanks to the authors and contributors of GRIT, wbc_fsm, motion_tracking,
[Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2),
[ONNX Runtime](https://github.com/microsoft/onnxruntime),
[yaml-cpp](https://github.com/jbeder/yaml-cpp),
[zlib](https://github.com/madler/zlib), and
[ZeroMQ](https://github.com/zeromq/libzmq).

## Features

- 50 Hz C++ inference and low-level control on the G1 computer
- Loco velocity tracking from Unitree remote or PICO sticks
- PICO VR whole-body tracking
- Single-file or directory-based NPZ playback
- Standing transitions between Loco, VR Tracking, and NPZ modes
- Optional voice announcements, enabled by default
- Persistent LED indication: green for Loco, blue for VR, purple for NPZ
- Command timeout, VR timeout, duplicate-bridge protection, and damping exit

The repository contains both the G1-side runtime and the workstation-side PICO
retargeting code. Follow
[Workstation and PICO setup](docs/workstation_setup.md) before using VR mode.

## Deployment roles and network

| Device | Runs |
| --- | --- |
| PICO | XRoboToolkit client and motion tracking |
| PC Server (workstation) | XRoboToolkit PC Service and the VR retargeting server |
| G1 Onboard | C++ robot bridge, ONNX inference, and low-level control |

For VR operation, **PICO, PC Server, and G1 must be on the same LAN and must be
able to reach one another**. `WORKSTATION_IP` is the PC Server's address on
this shared LAN.

## Project structure

```text
GRIT_Deploy_Onboard_G1/
├── g1_sim2real/
│   ├── config/                 # Bridge configuration
│   ├── scripts/                # Build and launch scripts
│   ├── src/                    # C++ bridge and inference runtime
│   └── third_party/            # Unitree SDK2, yaml-cpp, zlib
├── onnxruntime/                # ONNX Runtime 1.23.2, ARM64
├── sim2real/
│   ├── checkpoints/            # WBC and Loco ONNX models
│   ├── config/g1/              # Controller, robot and retarget configuration
│   ├── src/paths.py            # Shared workstation path resolution
│   ├── teleop/                 # PICO retargeting and ZMQ server
│   ├── install_xrobottoolkit_sdk.sh
│   ├── pyproject.toml
│   └── uv.lock
├── motions/                    # Five example NPZ trajectories
├── docs/                       # Workstation setup
├── THIRD_PARTY.md
└── LICENSE
```

## Requirements

- CMake 3.16+ and a C++17 compiler
- `make`, `flock`, pthreads, and `libzmq3-dev`
- For VR: PICO, PC Server, and G1 on the same low-latency LAN, with TCP ports
  `28701`, `28702`, and `28703` reachable
- For the PC Server: Ubuntu 22.04 or 24.04 x86_64, Python 3.10, and `uv`

Unitree SDK2, yaml-cpp, zlib, ARM64 ONNX Runtime, and both policy models are
included.

## Installation

### PC Server (workstation, required for VR)

Clone the repository and prepare the Python environment on the PC Server:

```bash
cd ~
git clone https://github.com/OWNER/GRIT_Deploy_Onboard_G1.git
cd ~/GRIT_Deploy_Onboard_G1/sim2real
uv sync
bash install_xrobottoolkit_sdk.sh
```

Then install and configure XRoboToolkit PC Service and the PICO client by
following [Workstation and PICO setup](docs/workstation_setup.md). This PC-side
environment is only required when VR Tracking is used.

### G1 Onboard

On the G1:

```bash
cd /home/unitree
git clone https://github.com/OWNER/GRIT_Deploy_Onboard_G1.git
cd /home/unitree/GRIT_Deploy_Onboard_G1
```

Install G1-side system dependencies when required:

```bash
sudo apt update
sudo apt install build-essential cmake libzmq3-dev
```

Build the onboard C++ runtime on the G1:

```bash
ONNXRUNTIME_ROOT="$PWD/onnxruntime" \
  bash g1_sim2real/scripts/build_onboard.sh
```

## Usage

### PC Server: start VR services

VR operation requires two PC Server terminals. Start XRoboToolkit PC Service:

```bash
cd /opt/apps/roboticsservice
bash runService.sh
```

Start the retargeting server in a second terminal:

```bash
cd ~/GRIT_Deploy_Onboard_G1/sim2real
taskset -c 1 uv run teleop/serve_xrobot_teleop.py --robot g1
```

The PC Server steps are not required for Loco + NPZ operation without VR.

### G1 Onboard: choose and start a control configuration

The bundled trajectories are:

- `boxing1.npz`
- `boxing2.npz`
- `boxing3.npz`
- `kick1.npz`
- `kpopdance.npz`

#### Loco + one NPZ trajectory

```bash
cd /home/unitree/GRIT_Deploy_Onboard_G1
bash g1_sim2real/scripts/run_npz_onboard.sh \
  --net <NET_INTERFACE> \
  --motion-file motions/boxing1.npz
```

#### Loco + NPZ playlist

```bash
cd /home/unitree/GRIT_Deploy_Onboard_G1
bash g1_sim2real/scripts/run_npz_onboard.sh \
  --net <NET_INTERFACE> \
  --motion-file motions/
```

Files are sorted by filename, so `boxing1.npz` is selected first. Each motion
plays once and then returns to the WBC standing pose.

#### Loco + VR

```bash
cd /home/unitree/GRIT_Deploy_Onboard_G1
bash g1_sim2real/scripts/run_vr_onboard.sh \
  --net <NET_INTERFACE> \
  --vr-server <WORKSTATION_IP>
```

#### Loco + VR + NPZ

```bash
cd /home/unitree/GRIT_Deploy_Onboard_G1
bash g1_sim2real/scripts/run_vr_onboard.sh \
  --net <NET_INTERFACE> \
  --vr-server <WORKSTATION_IP> \
  --motion-file motions/
```

For VR, PICO, the PC Server, and G1 must remain on the same reachable LAN.
`WORKSTATION_IP` must be the PC Server address on that LAN, and the PICO client
must use the same address for `PC Service`. The G1 `--net <NET_INTERFACE>`
option selects its DDS control interface. Voice is enabled by default; add
`--voice-announcements false` to disable voice announcements.

## Controls

After launch, center all sticks and press Unitree remote `START` to enter Loco.
In the three-mode configuration, remote or PICO `B` cycles:

```text
Loco -> VR Tracking -> NPZ -> Loco
```

Each mode handoff passes through a controlled standing transition.

### Unitree remote

| Input | Action |
| --- | --- |
| `START` | Activate from startup; return to stand in VR/NPZ |
| `B` | Switch to the next mode |
| `A` | Play/restart the selected trajectory in NPZ mode |
| D-pad left/right | Previous/next trajectory while standing in NPZ mode |
| Left stick | Loco forward and lateral velocity |
| Right stick horizontal | Loco yaw velocity |
| `X` | Damping emergency stop and program exit |

### PICO controllers

| Input | Action |
| --- | --- |
| `B` | Switch to the next mode |
| `A` | Start/re-align VR tracking, or play/restart NPZ |
| `X` | Return VR/NPZ to the WBC standing pose |
| `Y` | Damping emergency stop and program exit |
| Sticks | Loco velocity; previous/next trajectory while standing in NPZ |

The selected NPZ filename without its suffix is announced by voice. Keyboard
equivalents are `s` = `START`, `a` = `A`, `b` = `B`, `[`/`]` = previous/next,
`x` = remote `X`, and `q` = immediate bridge exit.

## Mode feedback

| Mode | Announcement | LED |
| --- | --- | --- |
| Loco | `loco mode` | Green `(0, 255, 0)` |
| VR Tracking | `VR Tracking mode` | Blue `(0, 0, 255)` |
| NPZ | `NPZ mode` | Purple `(128, 0, 255)` |

The active color is refreshed once per second and restored after speech.

## NPZ format

Trajectories must run at 50 Hz and contain finite `float32` or `float64`
C-order arrays:

| Field | Shape |
| --- | --- |
| `fps` | scalar `50` |
| `joint_pos` | `[T, 29]` |
| `joint_vel` | `[T, 29]` |
| `body_pos_w` | `[T, B, 3]` |
| `body_quat_w` | `[T, B, 4]` |
| `body_lin_vel_w` | `[T, B, 3]` |
| `body_ang_vel_w` | `[T, B, 3]` |

## Safety

- Keep at least **3 m** of unobstructed space around the robot.
- Use a gantry or physical support for every new model, configuration, or
  trajectory.
- Test the exact model and trajectory in simulation before hardware use.
- Check joint order, gains, network interface, battery, floor traction, model
  hashes, and neutral sticks before pressing `START`.
- Keep a trained operator at the physical emergency stop and do not rely only
  on software, voice, LED, or SSH.
- Remote `X` and PICO `Y` request damping and exit. A missing bridge state also
  triggers damping; stale VR data returns to WBC stand.

## License

Project code is released under [LICENSE](LICENSE). Models and dependencies
remain subject to their upstream terms. Exact versions, hashes, and notices are
listed in [THIRD_PARTY.md](THIRD_PARTY.md). Permission to redistribute both
included ONNX models with this project has been confirmed.
