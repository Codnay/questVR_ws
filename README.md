<div align="center">
  <h1 align="center"> quest teleop piper </h1>
  <h3 align="center"> Agilex Robotics </h3>
  <p align="center">
    <a>English</a> | <a href="README_CN.md">中文</a>
  </p>
</div>




## Overview

This repository implements teleoperation of the Piper robotic arm using a Meta Quest 2/3 VR headset.

### Prerequisites

**1. Install dependencies**

```bash
sudo apt install android-tools-adb

conda create -n vt python=3.9

conda activate vt

conda install pinocchio==3.2.0 casadi==3.6.7 -c conda-forge

pip install meshcat rospkg pyyaml pure-python-adb piper-sdk
```

**2. Enable Developer Mode (required to install third-party APKs)**

Before you start, confirm that Developer Mode is enabled on your Quest:

Settings → Advanced → Developer → enable “Developer settings”.

If you already have the Developer option, you can skip the activation steps below.

If you don’t see a Developer option, activate it as follows:

1) Register a Meta developer account  
→ Visit the Meta developer platform, sign in with your Meta account, create an organization (any name), and complete verification (may require a payment method).

2) Enable Developer Mode in the mobile app  
→ Meta Quest App → Device Settings → Developer Mode → turn on.

3) Allow unknown sources on the headset  
→ Headset Settings → System → Developer Options → enable “Unknown Sources”.

**3. Increase headset sleep timeout**

Set the sleep/display-off timeout to the maximum to avoid the headset turning off and stopping pose output.

→ Headset Settings → General → Power → set “Display off” to 4 hours.

**4. Install the APK on the headset (two methods)**

If you are in a region that requires it, make sure you have a stable network/proxy so the device can activate and reach Meta services.

1) Offline (recommended)

- Connect: enable Developer Mode, connect Quest to your PC via USB, then accept “Allow USB debugging”.
- Run:

```bash
adb install /path/to/teleop-debug.apk
```

Wait until the terminal prints `Success`.

2) With a stable network/proxy

- Install `teleop-debug.apk` on the Quest. The file is under `questVR_ws/src/oculus_reader/APK`.

  - Step 1: Install “Mobile VR Station” from the Meta store (requires internet).

  - Step 2: Connect Quest to the PC, enable USB debugging, then copy the APK into the Quest `Download` folder.

  - Step 3: Mobile VR Station → Configuration Wizard → Show All Options → Configure Scoped Storage → Step 1: Request Access → select the `Download` directory and install the APK.

- If the above fails, you can also use SideQuest (Windows required in some setups):  
  - <https://www.bilibili.com/opus/267781439861047911>  
  - <https://github.com/SideQuestVR/SideQuest/releases>

**5. Clone and build**

```bash
git clone git@github.com:agilexrobotics/questVR_ws.git

cd questVR_ws

catkin_make
```

Tested on Ubuntu 20.04. Other OS versions may require different configuration.

For more information, see [Getting started with Meta Quest 2](https://www.meta.com/zh-tw/help/quest/articles/getting-started/getting-started-with-quest-2/?srsltid=AfmBOoqvDcwTtPt2P9o6y3qdXT_9zxz4m8yyej4uwLGEXVXv6KAr3QQz), [Piper_ros](https://github.com/agilexrobotics/Piper_ros), and [oculus_reader](https://github.com/rail-berkeley/oculus_reader).

**6. Fix absolute mesh paths in the URDF**

In `piper_description`, update absolute paths inside `urdf/piper_description.urdf` to match your username, e.g.:

```xml
<geometry>
        <mesh
          filename="/home/agilex/questVR_ws/src/Piper_ros/src/piper_description/meshes/base_link.STL" />
</geometry>

Change to:

<geometry>
        <mesh
          filename="/home/<your name>/questVR_ws/src/Piper_ros/src/piper_description/meshes/base_link.STL" />
</geometry>
```

**7. Connect the PC and Quest via USB-C**

By default, we use a wired connection for stable, low-latency data transfer. If you need wireless, see [Wireless connection](#wireless-connection).

### Code structure

`oculus_reader` provides tools to read pose and button states from a Quest device.

It treats the headset as the base frame and forwards controller TFs (relative to the headset/base) to the robot arm.

```bash
├── APK    # APK files
│   ├── alvr_client_android.apk
│   └── teleop-debug.apk
├── CMakeLists.txt
├── config
│   └── oculus_reader.rviz
├── launch	# launch files
│   ├── teleop_double_piper.launch
│   └── teleop_single_piper.launch
├── package.xml
└── scripts
    ├── buttons_parser.py
    ├── FPS_counter.py
    ├── install.py
    ├── oculus_reader.py
    ├── piper_control.py		# arm control interface
    ├── teleop_double_piper.py	# dual-arm teleop
    ├── teleop_single_piper.py	# single-arm teleop
    └── tools.py
```

## Running

### 1) Enable the robot arm(s)

**Single Piper**

Connect the arm CAN cable to the computer, then run:

```bash
cd ~/questVR_ws/src/Piper_ros

bash can_activate.sh can0 1000000
```

**Dual Piper (left + right)**

First connect the left arm CAN cable, then run:

```bash
cd ~/questVR_ws/src/Piper_ros

bash find_all_can_port.sh
```

The terminal will show the left arm port. Then connect the right arm CAN cable.

Run again:

```bash
bash find_all_can_port.sh
```

Now you should have both ports.

Copy the left/right port IDs into `can_config.sh` (lines 111 and 112), for example:

```bash
# Predefined USB ports, target interface name, and bitrate (for multiple CAN modules)
if [ "$EXPECTED_CAN_COUNT" -ne 1 ]; then
    declare -A USB_PORTS
    USB_PORTS["1-8.1:1.0"]="left_piper:1000000"  # left arm
    USB_PORTS["1-8.2:1.0"]="right_piper:1000000" # right arm
fi
```

After saving, activate both arms:

```bash
cd ~/questVR_ws/src/Piper_ros

bash can_config.sh
```

### 2) Start teleoperation

```bash
source /home/agilex/questVR_ws/devel/setup.bash

conda activate vt

roslaunch oculus_reader teleop_single_piper.launch    # single-arm teleop

or

roslaunch oculus_reader teleop_double_piper.launch    # dual-arm teleop
```

If you see the error below when launching:

```bash
Device not found. Make sure that device is running and is connected over USB
Run `adb devices` to verify that the device is visible.
```

It means USB debugging hasn’t been authorized on the headset:

1. Connect the Quest to the computer via USB-C and put on the headset.
2. When you see the “USB detected” notification, click it.

   ![img error](img/2.png)

3. The first run may still show the error above.
4. When prompted on the headset, accept **Allow USB debugging** and **Always allow from this computer**.

   ![img error](img/1.png)

5. Quit the program and run it again.

## Operation notes

> Notes:
>
> - Keep the VR display awake. If tracking is lost, TF can drift and cause unsafe arm motion. A common trick is to cover the proximity sensor so the headset stays awake.
> - After starting the program, ensure the controllers are tracked (in view) and the RViz frames are stable (not drifting). Then hold “A” (right) or “X” (left) to reset the arm before teleoperating.
> - After teleop starts, watch the visualization/web UI to confirm the robot is not drifting.

- For single-arm teleop, use the right controller. Before teleop, make sure the arm is in the initial pose. Hold “A” to return the arm to the initial position. Hold “B” to teleoperate; release to stop. Dual-arm teleop is similar.

- For safety and to reduce wear, return the arm close to the initial pose before pressing “A” or “X” to reset at the end of teleoperation.

## Controller button reference

You can read button values with:

```bash
transformations, buttons = oculus_reader.get_transformations_and_buttons()
```

Example output from `print("buttons:", buttons)`:

```python
buttons: {'A': False, 'B': False, 'RThU': True, 'RJ': False, 'RG': False, 'RTr': False, 'X': False, 'Y': False, 'LThU': True, 'LJ': False, 'LG': False, 'LTr': False, 'leftJS': (0.0, 0.0), 'leftTrig': (0.0,), 'leftGrip': (0.0,), 'rightJS': (0.0, 0.0), 'rightTrig': (0.0,), 'rightGrip': (0.0,)}
```

### Button states (booleans: `True`/`False`)

These fields are booleans (`True`/`False`). `False` means “not pressed”, `True` means “pressed”.

- **`'A': False`**: Right controller “A” button is not pressed.
- **`'B': False`**: Right controller “B” button is not pressed.
- **`'X': False`**: Left controller “X” button is not pressed.
- **`'Y': False`**: Left controller “Y” button is not pressed.
- **`'RThU': True`**: **R**ight **Th**umbstick **U**p. Your right thumb is on the capacitive sensor, but the stick is not clicked.
- **`'LThU': True`**: **L**eft **Th**umbstick **U**p. Your left thumb is on the capacitive sensor, but the stick is not clicked.
- **`'RJ': False`**: **R**ight **J**oystick click is not pressed.
- **`'LJ': False`**: **L**eft **J**oystick click is not pressed.
- **`'RG': False`**: **R**ight **G**rip is not pressed.
- **`'LG': False`**: **L**eft **G**rip is not pressed.
- **`'RTr': False`**: **R**ight **Tr**igger is not fully pressed (often treated as `True` only past a threshold).
- **`'LTr': False`**: **L**eft **Tr**igger is not fully pressed.

### Analog values (tuples of floats)

These fields are float tuples representing joystick deflection or trigger/grip depth (typically 0.0–1.0 or -1.0–1.0).

- **`'leftJS': (0.0, 0.0)`**: Left joystick state `(x, y)`. `(0.0, 0.0)` means centered.
- **`'rightJS': (0.0, 0.0)`**: Right joystick state `(x, y)`. `(0.0, 0.0)` means centered.
- **`'leftTrig': (0.0,)`**: Left trigger depth. `0.0` released, `1.0` fully pressed.
- **`'rightTrig': (0.0,)`**: Right trigger depth. `0.0` released, `1.0` fully pressed.
- **`'leftGrip': (0.0,)`**: Left grip depth. `0.0` released, `1.0` fully pressed.
- **`'rightGrip': (0.0,)`**: Right grip depth. `0.0` released, `1.0` fully pressed.

## Wireless connection

### Phase 1: Preparation (same LAN required)

- **Same Wi‑Fi**: ensure your computer and Quest are connected to the same router.
- **Prefer 5GHz**: to reduce latency and packet loss, use a 5GHz network rather than 2.4GHz.
- **ADB installed**: ensure `adb` is installed on the computer.

### Phase 2: First connection (enable wireless ADB)

After a reboot, Quest typically disables wireless debugging. After every full power cycle/reboot, you usually need to do this once:

1. **USB cable**: connect Quest to the computer via USB.
2. **Authorize**: put on the headset and accept the “Allow USB debugging” prompt (check “Always allow”).
3. **Switch ADB to TCP mode**:

   ```bash
   adb tcpip 5555
   ```

   *On success, you’ll see: `restarting in TCP mode port: 5555`.*

4. **Unplug USB**: you can now disconnect the cable.

### Phase 3: Get IP and connect

1. **Find the Quest IP address**:

   - **Method A (on headset)**: Settings → Wi‑Fi → select the connected Wi‑Fi → Details → note the IP address.
   - **Method B (on computer)**:

     ```bash
     adb shell ip route
     ```

     *Look for the `src` value for `wlan0`.*

2. **Connect over Wi‑Fi**:

   ```bash
   adb connect <your_Quest_IP>:5555
   # Example: adb connect 192.168.1.101:5555
   ```

   *If you see `connected to ...`, wireless ADB is working.*

### Phase 4: Use from Python

Pass the same IP to `OculusReader`:

```Python
from oculus_reader import OculusReader

# Make sure this IP matches the one used in adb connect
self.oculus_reader = OculusReader(ip_address='192.168.1.101')
```

### Phase 5: Run

`oculus_reader` relies on an APK installed on the Quest to collect sensor data.

1. **Confirm the APK is installed**
2. **Start the program**
   - Follow [Running](#running) to launch.
   - You may be prompted again to allow USB debugging; check “Always allow” and confirm.
