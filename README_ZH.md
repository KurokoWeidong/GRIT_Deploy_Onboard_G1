# GRIT Deploy Onboard G1

Unitree G1 端侧 C++ 部署项目，支持 Loco 速度跟踪、VR 全身遥操作和 NPZ 轨迹
播放。

[English](README.md) | 中文

## 引用与致谢

本项目主要基于 [GRIT](https://github.com/mrzuang/GRIT_teleop_deploy)，全身运控
（WBC）模型
[`policy.onnx`](https://github.com/mrzuang/GRIT_teleop_deploy/blob/main/sim2real/checkpoints/policy.onnx)
来源于 GRIT。

Loco 速度跟踪模型使用
[`Unitree-G1-AMP-Flat_model_30000.onnx`](https://github.com/ccrpRepo/wbc_fsm/blob/main/model/loco/Unitree-G1-AMP-Flat_model_30000.onnx)，
来源于 [ccrpRepo/wbc_fsm](https://github.com/ccrpRepo/wbc_fsm)。

机器人桥接和 VR 重定向设计参考了
[Axellwppr/motion_tracking 的 sim2real 分支](https://github.com/Axellwppr/motion_tracking/tree/sim2real)。

感谢 GRIT、wbc_fsm、motion_tracking，以及
[Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)、
[ONNX Runtime](https://github.com/microsoft/onnxruntime)、
[yaml-cpp](https://github.com/jbeder/yaml-cpp)、
[zlib](https://github.com/madler/zlib) 和
[ZeroMQ](https://github.com/zeromq/libzmq) 的作者与贡献者。

## 功能

- 在 G1 内部计算单元上运行 50 Hz C++ 推理和底层控制
- 通过宇树遥控器或 PICO 摇杆进行 Loco 速度控制
- PICO VR 全身动作跟踪
- 播放单条 NPZ 或文件夹内的轨迹列表
- Loco、VR Tracking、NPZ 三模式站立过渡切换
- 默认开启、可关闭的语音播报
- 持续 LED 指示：Loco 绿色、VR 蓝色、NPZ 紫色
- 指令超时、VR 超时、桥接器防重复启动和阻尼退出保护

本仓库同时包含 G1 端侧运行时和工作站端 PICO 重定向代码。使用 VR 模式前请按照
[工作站与 PICO 配置](docs/workstation_setup_zh.md)进行设置。

## 部署角色与网络

| 设备 | 运行内容 |
| --- | --- |
| PICO | XRoboToolkit 客户端与动作追踪 |
| PC Server（工作站） | XRoboToolkit PC Service 与 VR 动作重定向服务 |
| G1 Onboard（机器人端） | C++ 机器人桥接、ONNX 推理与底层控制 |

使用 VR 功能时，**PICO、PC Server 和 G1 必须处于同一局域网，并且三者能够互相
访问**。`WORKSTATION_IP` 是 PC Server 在该局域网中的地址。

## 项目结构

```text
GRIT_Deploy_Onboard_G1/
├── g1_sim2real/
│   ├── config/                 # 桥接配置
│   ├── scripts/                # 编译和启动脚本
│   ├── src/                    # C++ 桥接与推理运行时
│   └── third_party/            # Unitree SDK2、yaml-cpp、zlib
├── onnxruntime/                # ARM64 ONNX Runtime 1.23.2
├── sim2real/
│   ├── checkpoints/            # WBC 和 Loco ONNX 模型
│   ├── config/g1/              # 控制器、机器人与重定向配置
│   ├── src/paths.py            # 工作站路径解析
│   ├── teleop/                 # PICO 重定向和 ZMQ 服务
│   ├── install_xrobottoolkit_sdk.sh
│   ├── pyproject.toml
│   └── uv.lock
├── motions/                    # 5 条示例 NPZ 轨迹
├── docs/                       # 工作站配置说明
├── THIRD_PARTY.md
└── LICENSE
```

## 环境要求

- CMake 3.16+ 和支持 C++17 的编译器
- `make`、`flock`、pthreads 和 `libzmq3-dev`
- VR 模式要求 PICO、PC Server 和 G1 处于同一低延迟局域网，并能访问 TCP 端口
  `28701`、`28702`、`28703`
- PC Server 支持 Ubuntu 22.04 或 24.04 x86_64，需要 Python 3.10 和 `uv`

项目已包含 Unitree SDK2、yaml-cpp、zlib、ARM64 ONNX Runtime 和两个策略模型。

## 安装

### PC Server（工作站，VR 模式需要）

在 PC Server 上克隆项目并准备 Python 环境：

```bash
cd ~
git clone https://github.com/OWNER/GRIT_Deploy_Onboard_G1.git
cd ~/GRIT_Deploy_Onboard_G1/sim2real
uv sync
bash install_xrobottoolkit_sdk.sh
```

然后按照[工作站与 PICO 配置](docs/workstation_setup_zh.md)安装 XRoboToolkit PC
Service 并配置 PICO 客户端。只有使用 VR Tracking 时才需要这套 PC 端环境。

### G1 Onboard（机器人端）

在 G1 上：

```bash
cd /home/unitree
git clone https://github.com/OWNER/GRIT_Deploy_Onboard_G1.git
cd /home/unitree/GRIT_Deploy_Onboard_G1
```

按需安装 G1 端系统依赖：

```bash
sudo apt update
sudo apt install build-essential cmake libzmq3-dev
```

在 G1 上编译端侧 C++ 程序：

```bash
ONNXRUNTIME_ROOT="$PWD/onnxruntime" \
  bash g1_sim2real/scripts/build_onboard.sh
```

## 使用方法

### PC Server：启动 VR 服务

使用 VR 功能时，需要在 PC Server 上打开两个终端。终端 1 启动 XRoboToolkit
PC Service：

```bash
cd /opt/apps/roboticsservice
bash runService.sh
```

终端 2 启动动作重定向服务：

```bash
cd ~/GRIT_Deploy_Onboard_G1/sim2real
taskset -c 1 uv run teleop/serve_xrobot_teleop.py --robot g1
```

仅使用 Loco + NPZ、不启用 VR 时，无需执行上述 PC Server 启动步骤。

### G1 Onboard：选择并启动控制配置

项目随附以下轨迹：

- `boxing1.npz`
- `boxing2.npz`
- `boxing3.npz`
- `kick1.npz`
- `kpopdance.npz`

#### Loco + 单条 NPZ

```bash
cd /home/unitree/GRIT_Deploy_Onboard_G1
bash g1_sim2real/scripts/run_npz_onboard.sh \
  --net <NET_INTERFACE> \
  --motion-file motions/boxing1.npz
```

#### Loco + NPZ 文件夹

```bash
cd /home/unitree/GRIT_Deploy_Onboard_G1
bash g1_sim2real/scripts/run_npz_onboard.sh \
  --net <NET_INTERFACE> \
  --motion-file motions/
```

文件按名称排序，因此使用随附的 `motions/` 时默认选中 `boxing1.npz`。每条轨迹
只播放一遍，完成后自动返回 WBC 站姿。

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

使用 VR 时，PICO、PC Server 和 G1 必须始终处于同一可互访的局域网。
`WORKSTATION_IP` 必须填写 PC Server 在该局域网中的地址，PICO 客户端的
`PC Service` 也应填写同一地址。G1 端的 `--net <NET_INTERFACE>` 用于选择 DDS 控制网卡。语音默认开启；增加 `--voice-announcements false` 可以
关闭语音播报。

## 控制方式

程序启动后，将所有摇杆回中并按宇树遥控器 `START` 进入 Loco。三模式配置下，
遥控器或 PICO 的 `B` 按以下顺序切换：

```text
Loco -> VR Tracking -> NPZ -> Loco
```

每次切换都会经过受控站立过渡状态。

### 宇树遥控器

| 按键/输入 | 功能 |
| --- | --- |
| `START` | 首次启动控制；在 VR/NPZ 中回到 WBC 站姿 |
| `B` | 切换到下一模式 |
| `A` | 在 NPZ 模式播放/从头重播当前轨迹 |
| 十字键左/右 | 在 NPZ 稳定站姿下选择上一条/下一条轨迹 |
| 左摇杆 | Loco 前后和侧向速度 |
| 右摇杆横向 | Loco 转向角速度 |
| `X` | 阻尼急停并退出程序 |

### PICO 手柄

| 按键/输入 | 功能 |
| --- | --- |
| `B` | 切换到下一模式 |
| `A` | 开始/重新对齐 VR，或播放/从头重播 NPZ |
| `X` | 在 VR/NPZ 中回到 WBC 站姿 |
| `Y` | 阻尼急停并退出程序 |
| 摇杆 | Loco 速度控制；在 NPZ 稳定站姿下切换轨迹 |

选中 NPZ 后会播报不含后缀的完整文件名。键盘等价按键为：`s` = `START`、
`a` = `A`、`b` = `B`、`[`/`]` = 上一条/下一条、`x` = 遥控器 `X`、
`q` = 立即退出桥接器。

## 模式反馈

| 模式 | 语音 | LED |
| --- | --- | --- |
| Loco | `loco mode` | 绿色 `(0, 255, 0)` |
| VR Tracking | `VR Tracking mode` | 蓝色 `(0, 0, 255)` |
| NPZ | `NPZ mode` | 紫色 `(128, 0, 255)` |

当前模式颜色每秒刷新一次，并在语音播报后恢复。

## NPZ 格式

轨迹必须为 50 Hz，并包含有限数值的 NumPy `float32` 或 `float64` C-order 数组：

| 字段 | 形状 |
| --- | --- |
| `fps` | 标量 `50` |
| `joint_pos` | `[T, 29]` |
| `joint_vel` | `[T, 29]` |
| `body_pos_w` | `[T, B, 3]` |
| `body_quat_w` | `[T, B, 4]` |
| `body_lin_vel_w` | `[T, B, 3]` |
| `body_ang_vel_w` | `[T, B, 3]` |

## 安全要求

- 机器人周围必须保持至少 **3 米**无障碍空间。
- 每个新模型、新配置或新轨迹首次运行时必须使用吊装或其他物理保护。
- 真机运行前必须在仿真中验证完全相同的模型和轨迹。
- 按 `START` 前检查关节顺序、控制增益、网卡、电池、地面防滑、模型哈希和
  摇杆回中状态。
- 必须安排操作人员掌握物理急停，不得只依赖软件、语音、LED 或 SSH。
- 遥控器 `X` 和 PICO `Y` 会请求阻尼并退出；桥接状态丢失也会触发阻尼，VR
  数据超时会返回 WBC 站姿。

## 许可证

项目代码使用 [LICENSE](LICENSE) 中的许可证。模型和第三方依赖仍受各自条款
约束，具体版本、哈希和声明见 [THIRD_PARTY.md](THIRD_PARTY.md)。两个随项目提供的
ONNX 模型均已确认获得随本项目重新分发的授权。
