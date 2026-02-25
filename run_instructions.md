# Minimal Run Instructions (Ubuntu 24.04 Host + Docker)

This repo is a ROS1 (catkin) workspace. On Ubuntu 24.04, install/run it via Docker (ROS Noetic / Ubuntu 20.04 inside the container).

## 0) One-Time Setup (Quest + ADB)

- Enable Quest developer mode + USB debugging.
- On the host, verify ADB works and the device is authorized:
  - `adb kill-server && adb start-server`
  - `adb devices -l` should show your Quest as `device` (not `no permissions` / `unauthorized`).
- Install the APK from this repo (host-side):
  - `adb install -r src/oculus_reader/APK/teleop-debug.apk`

Note: `oculus_reader` uses a Python ADB client that talks to the ADB server on `127.0.0.1:5037`, so keep the host ADB server running.

## 1) One-Time Setup (Build Docker Image)

From the workspace root:

- `cd ~/Desktop/Robotics/questVR_ws`
- `docker build -f Dockerfile.noetic -t questvr_noetic:latest .`

## 2) Build The Catkin Workspace (Inside Docker)

If `build/` and `devel/` are not already present:

- `docker run --rm -it --net=host --privileged -v "$PWD":/ws -w /ws questvr_noetic:latest bash`
- Inside the container:
  - `source /opt/ros/noetic/setup.bash`
  - `catkin_make`

## 3) Bring Up CAN On The Host (Required For Piper)

`piper_ctrl_single_node` expects a SocketCAN interface named `can0` at `1000000` bps.

- Plug in your USB-CAN adapter and connect the Piper CAN cable.
- Ensure `can0` exists and is up at 1Mbps (repo provides scripts):
  - `cd ~/Desktop/Robotics/questVR_ws/src/Piper_ros`
  - `bash find_all_can_port.sh`
  - `bash can_activate.sh can0 1000000`

If `find_all_can_port.sh` prints nothing and `can_activate.sh` reports `CAN 模块数量 (0)`, Linux is not seeing your USB-CAN adapter yet.

## 4) Run Teleop (Headless, Inside Docker)

From the workspace root:

- `cd ~/Desktop/Robotics/questVR_ws`
- `docker run --rm -it --net=host --privileged -v "$PWD":/ws -w /ws questvr_noetic:latest bash -lc 'source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; roslaunch oculus_reader teleop_single_piper_headless.launch'`

If your CAN interface name is not `can0` (e.g. `can1`), pass it as:

- `docker run --rm -it --net=host --privileged -v "$PWD":/ws -w /ws questvr_noetic:latest bash -lc 'source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; roslaunch oculus_reader teleop_single_piper_headless.launch can_port:=can1'`

What to expect:

- If it starts correctly, you should see Meshcat print a URL like `http://127.0.0.1:7000/static/`.
- If you see a CAN error like `FileNotFoundError(2, 'No such file or directory')`, your `can0` interface is not present/up on the host.
