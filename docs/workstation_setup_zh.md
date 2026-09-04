# 工作站与 PICO 配置

[English](workstation_setup.md) · [返回中文 README](../README_ZH.md)

本文用于配置 VR 遥操作的工作站端。

> **网络要求：**PICO、PC Server（工作站）和 G1 必须处于同一局域网，并且三者
> 能够互相访问。PICO 中配置的地址和 G1 端传入的 `WORKSTATION_IP`，均为 PC
> Server 在该共享局域网中的地址。

## 1. 硬件与网络

准备以下设备和环境：

- Ubuntu 22.04 或 24.04 x86_64 工作站；
- PICO 4 或 PICO 4 Pro、两个手柄和两个 PICO Motion Tracker；
- PC Server、PICO 和 G1 均可访问的低延迟 5 GHz 局域网；
- 可选的独立 G1 有线管理连接。

条件允许时，建议工作站通过网线连接无线路由器。PICO 数据网络与 G1 专用控制或
管理网口应使用不同网段，避免系统选择错误路由。

## 1. 安装 XRoboToolkit PC Service

下载并安装 XRoboToolkit PC Service。下面示例使用官方文件名中带有
`ubuntu_22.04` 的软件包；在 Ubuntu 24.04 上，应优先使用上游明确兼容 24.04 的
版本。

```bash
wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```

Ubuntu 24.04、其他系统或新版本请查看
[XRoboToolkit PC Service Releases](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases)。

把修订版 XRoboToolkit Python 绑定安装到 GRIT 环境：

```bash
cd ~/GRIT_Deploy_Onboard_G1/sim2real
bash install_xrobottoolkit_sdk.sh
```

## 2. 安装和标定 PICO 客户端

1. 在头显浏览器中安装
   [XRoboToolkit PICO 应用](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases)，
   然后从未知来源应用列表中打开。
2. 将两个 Motion Tracker 分别固定在左右脚踝，检查方向和左右位置。
3. 配对两个追踪器，双脚与肩同宽自然站立，并按头显提示完成标定。
4. 使用 `Calibrate Floor`，确认虚拟角色双脚正确落在地面上。
5. 更换操作人员、追踪器位置或场地后重新标定。

在 XRoboToolkit 中启用：

| 区域 | 必需设置 |
| --- | --- |
| `Tracking` | `Head` 和 `Controller` |
| `Data/Control` | `Send` |
| `Pico Motion Tracker` | `Full body` |

## 3. 配置工作站地址

查询工作站在 PICO 和 G1 所在 Wi-Fi/局域网中的 IPv4 地址：

```bash
cd ~/GRIT_Deploy_Onboard_G1
ip -4 addr show <WIFI_INTERFACE>
```

将该地址填入 PICO 应用的 `PC Service`，然后选择 `Reconnect`。连接成功后状态
必须显示 `WORKING`。

继续操作前，请确认 PICO、PC Server 和 G1 位于同一局域网并能互相访问，同时
关闭无线路由器的客户端隔离功能。

端侧启动时必须单独指定 `WORKSTATION_IP`。它是工作站在 VR 共享网络上的地址，
不能填写 G1 地址、`127.0.0.1`、容器地址或无关的有线网卡地址。PICO 的
`PC Service` 和 G1 的 `--vr-server` 必须使用同一个 PC Server 地址；G1 的
`--net <NET_INTERFACE>` 用于选择 DDS 网卡，是另一项独立配置。

## 4. 启动工作站服务

终端 1——启动 XRoboToolkit PC Service：

```bash
cd /opt/apps/roboticsservice
bash runService.sh
```

终端 2——启动 GRIT 动作重定向：

```bash
cd ~/GRIT_Deploy_Onboard_G1/sim2real
taskset -c 1 uv run teleop/serve_xrobot_teleop.py --robot g1
```

如果工作站没有 CPU 1，可以去掉 `taskset -c 1`。CPU 亲和性仅用于性能优化，
不是功能要求。

动作重定向服务使用 TCP `28701`、`28702`、`28703`，浏览器可视化使用 TCP
`8080`。在工作站的另一个终端检查监听状态：

```bash
cd ~/GRIT_Deploy_Onboard_G1/sim2real
ss -ltnp | grep -E '28701|28702|28703|8080'
```

打开 `http://localhost:8080`，确认人体姿态与重定向后的 G1 动作合理，再启用真机
控制。

## 5. 启动机器人端

将前面查询到的工作站 Wi-Fi/局域网地址填入 `<WORKSTATION_IP>`：

```bash
cd /home/unitree/GRIT_Deploy_Onboard_G1
bash g1_sim2real/scripts/run_vr_onboard.sh \
  --net eth0 \
  --vr-server <WORKSTATION_IP> \
  --motion-file motions/
```

只有在工作站服务正常、PICO 显示 `WORKING`、重定向画面合理、所有摇杆回中、
机器人已获得物理保护且周围 3 米净空后，才能按遥控器 `START`。

## 常见问题

### `ModuleNotFoundError: xrobotoolkit_sdk`

在当前 GRIT 环境中重新安装修订版绑定：

```bash
cd ~/GRIT_Deploy_Onboard_G1/sim2real
bash install_xrobottoolkit_sdk.sh
```

### PICO 未显示 `WORKING`

确认 PC Service 正在运行、设备处于同一网络、路由器未启用客户端隔离，并确认填写
的是工作站 Wi-Fi/局域网地址。

### 动作重定向日志一直为 `cb=0`

确认已启用 `Head`、`Controller`、`Send`、`Full body`，然后重新标定追踪器并
重新连接 PICO。

### 延迟或抖动明显

优先使用 5 GHz Wi-Fi，缩短与接入点的距离，停止并行视频流或大文件传输，并将
PICO/G1 路由与无关的容器或管理网络分开。
