# Workstation and PICO setup

[中文说明](workstation_setup_zh.md) · [Back to main README](../README.md)

This guide configures the workstation side of VR teleoperation.

> **Network requirement:** PICO, the PC Server (workstation), and G1 must be on
> the same LAN and must be able to reach one another. The address configured in
> PICO and passed to G1 as `WORKSTATION_IP` is the PC Server's address on this
> shared LAN.

## 1. Hardware and network

Prepare:

- an Ubuntu 22.04 or 24.04 x86_64 workstation;
- a PICO 4 or PICO 4 Pro, two controllers, and two PICO Motion Trackers;
- a low-latency 5 GHz LAN shared by the PC Server, PICO, and G1;
- an optional separate wired G1 management connection.

Connect the workstation to the access point by Ethernet when possible. Keep the
PICO data network and any dedicated G1 control/management interface on distinct
subnets to avoid route selection errors.

## 1. Install XRoboToolkit PC Service

Download and install the XRoboToolkit PC Service package. The following command
uses the official package whose filename contains `ubuntu_22.04`; on Ubuntu
24.04, use an upstream package explicitly compatible with 24.04 when available.

```bash
wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```

For Ubuntu 24.04, another operating system, or a newer release, check the
[XRoboToolkit PC Service releases](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases).

Install the patched XRoboToolkit Python binding into the GRIT environment:

```bash
cd ~/GRIT_Deploy_Onboard_G1/sim2real
bash install_xrobottoolkit_sdk.sh
```

## 2. Install and calibrate the PICO client

1. Install the
   [XRoboToolkit PICO application](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases)
   from the headset browser and open it from the unknown-sources application
   list.
2. Attach one Motion Tracker to each ankle and verify left/right orientation.
3. Pair both trackers and follow the headset calibration procedure while
   standing naturally with feet shoulder-width apart.
4. Use `Calibrate Floor` until the virtual character's feet rest on the floor.
5. Repeat calibration after changing the operator, tracker placement, or test
   area.

In XRoboToolkit enable:

| Section | Required setting |
| --- | --- |
| `Tracking` | `Head` and `Controller` |
| `Data/Control` | `Send` |
| `Pico Motion Tracker` | `Full body` |

## 3. Configure the workstation address

Identify the workstation's IPv4 address on the Wi-Fi/LAN shared with the PICO
and G1:

```bash
cd ~/GRIT_Deploy_Onboard_G1
ip -4 addr show <WIFI_INTERFACE>
```

Set this address as `PC Service` in the PICO application and select
`Reconnect`. The status must become `WORKING`.

Before continuing, confirm that PICO, the PC Server, and G1 are on the same LAN
and can reach one another. Disable wireless client isolation on the access point.

`WORKSTATION_IP` must be specified independently when the onboard launcher is
started. It is the workstation address on the shared VR network. Do not use the
G1 address, `127.0.0.1`, a container address, or an unrelated wired-interface
address. The PICO `PC Service` field and G1 `--vr-server` argument must use this
same PC Server address. G1's `--net <NET_INTERFACE>` selects its DDS interface
and is a separate setting.

## 4. Start the workstation services

Terminal 1 — start XRoboToolkit PC Service:

```bash
cd /opt/apps/roboticsservice
bash runService.sh
```

Terminal 2 — start GRIT retargeting:

```bash
cd ~/GRIT_Deploy_Onboard_G1/sim2real
taskset -c 1 uv run teleop/serve_xrobot_teleop.py --robot g1
```

If CPU 1 is unavailable, omit `taskset -c 1`. CPU affinity is an optimization,
not a functional requirement.

The retarget server uses TCP `28701`, `28702`, and `28703`. Its browser viewer
uses TCP `8080`. Check the listeners from another workstation terminal:

```bash
cd ~/GRIT_Deploy_Onboard_G1/sim2real
ss -ltnp | grep -E '28701|28702|28703|8080'
```

Open `http://localhost:8080` and verify that both the human pose and retargeted
G1 move correctly before enabling hardware control.

## 5. Start the onboard side

Use the Wi-Fi/LAN address identified above as `<WORKSTATION_IP>`:

```bash
cd /home/unitree/GRIT_Deploy_Onboard_G1
bash g1_sim2real/scripts/run_vr_onboard.sh \
  --net eth0 \
  --vr-server <WORKSTATION_IP> \
  --motion-file motions/
```

Do not press remote `START` until the workstation services are healthy, PICO
shows `WORKING`, the retarget view is plausible, all sticks are centered, and
the robot is physically supported in a clear 3 m safety area.

## Troubleshooting

### `ModuleNotFoundError: xrobotoolkit_sdk`

Reinstall the patched binding in the active GRIT environment:

```bash
cd ~/GRIT_Deploy_Onboard_G1/sim2real
bash install_xrobottoolkit_sdk.sh
```

### PICO does not show `WORKING`

Verify that the PC Service is running, both devices are on the same network,
client isolation is disabled, and the configured address is the workstation's
Wi-Fi/LAN address.

### The retarget log remains at `cb=0`

Verify `Head`, `Controller`, `Send`, and `Full body`; then recalibrate the
trackers and reconnect the PICO client.

### High latency or jitter

Prefer 5 GHz Wi-Fi, reduce distance to the access point, avoid concurrent video
or large file transfers, and keep the PICO/G1 route separate from unrelated
container or management routes.
