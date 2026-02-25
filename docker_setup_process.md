# Docker Setup Process (What Was Done + Why)

This documents the local Docker-based workflow used to run `questVR_ws` on Ubuntu 24.04.

## Why Docker Was Needed

- This repository is ROS1 + catkin (`catkin_make`, `roslaunch`).
- Upstream README notes testing on Ubuntu 20.04 (ROS Noetic).
- Ubuntu 24.04 does not provide ROS1 Noetic packages via `apt`, so host-side `roslaunch` is not available.

## What Was Tried / Observed

1. Built the workspace using `osrf/ros:noetic-desktop-full` with the workspace mounted at `/ws`.
2. Sourcing `devel/setup.*` on the Ubuntu 24.04 host failed because those setup files referenced the in-container path `/ws/...` (expected).
3. Running `roslaunch oculus_reader teleop_single_piper.launch` inside the container initially failed due to:
   - GUI/RViz issues (`could not connect to display`) when no X server is forwarded.
   - Missing Python deps used by the teleop nodes (`casadi`, `pinocchio`, `pure-python-adb` a.k.a. `ppadb`, `piper_sdk`).

## Changes Made In The Repo

- Added a headless launch file (no RViz):
  - `questVR_ws/src/oculus_reader/launch/teleop_single_piper_headless.launch`
- Updated URDF mesh paths for the container-mounted workspace:
  - `questVR_ws/src/Piper_ros/src/piper_description/urdf/piper_description.urdf`
  - Replaced `/home/agilex/questVR_ws/...` with `/ws/...` so Pinocchio can locate STL meshes when running inside Docker with `-v "$PWD":/ws`.
- Added a Dockerfile that installs the Python deps needed by the nodes:
  - `questVR_ws/Dockerfile.noetic`
  - Uses `osrf/ros:noetic-desktop-full` as the base.
  - Installs heavy numeric deps (`pinocchio=3.2.0`, `casadi=3.6.7`) via micromamba (conda-forge) in a Python 3.9 env.
  - Installs runtime Python deps via pip (`meshcat`, `pure-python-adb`, `piper-sdk`, etc.).

## Current Known Runtime Blockers (Hardware)

- CAN: `piper_ctrl_single_node` requires a host SocketCAN interface (default `can0`) at 1 Mbps.
  - If `can0` does not exist/up, the node fails with `FileNotFoundError(2, 'No such file or directory')`.
- Quest ADB: `oculus_reader` connects via ADB; ensure the host ADB server is running and the headset is authorized (`adb devices -l` shows `device`).

## How To Run With This Setup

See `questVR_ws/run_instructions.md`.

