# G1Pilot

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Ros Version](https://img.shields.io/badge/ROS1-Noetic-red)](
https://docs.ros.org/en/noetic/index.html)
[![Ros Version](https://img.shields.io/badge/ROS2-Humble-green)](
https://docs.ros.org/en/humble/index.html)

G1Pilot is an open‑source ROS 2 package for Unitree G1 humanoid robots. It exposes two complementary control paths—Joint (low‑level, per‑joint) and Cartesian (end‑effector) and continuously publishes core robot state for monitoring and visualization in RViz.

✳️ Use cases: teleoperation, manipulation research, motion analysis, and integration with higher‑level planners.

## Highlights

- Dual control modes: switch between Joint and Cartesian control on the fly.

- Always‑on telemetry: IMU, odometry, and per‑motor feedback (temperature, voltage, position, velocity).

- RViz‑ready: packaged URDF + RViz config for immediate visualization of the real robot.

- Docker‑first workflow: reproducible build/run scripts for Ubuntu 22.04 + ROS 2 Humble.

- Extensible: clear node boundaries and parameters make it easy to add behaviors or swap planners.


## Visual Overview
| **Joint Controller** | **Cartesian Controller** |
|---------------------|--------------------|
| <img src="https://github.com/hucebot/g1pilot/blob/main/images/joint_controller.gif" alt="Static Sensors" width="400"> | <img src="https://github.com/hucebot/g1pilot/blob/main/images/cartesian_controller.gif" alt="Moving Sensors" width="400"> |

## Table of Contents
- [Pre-requisites](#pre-requisites)
- [Quick Start](#quick-start)
- [Nodes Overview](#-nodes-overview)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Pre-requisites
- For visualization, you need to install the [g1pilot](https://github.com/hucebot/g1pilot) package in the same directory as this package.
- Be connected to the robot via WiFi or Ethernet. **It's important to know which interface you are using.**

## Quick Start
### Docker (recommended)
We prepare a docker image to build the package. You can use the following command to build the package, go the `docker` folder and run the following command:

```bash
sh build.sh
```

Then, you can run the docker image with the following command:

```bash
sh run.sh
```

### Package Layout

``` bash
.
├─ g1pilot/                 # Python nodes
├─ description_files/       # URDF/Xacro, meshes
├─ config/                  # RViz and node configs
├─ launch/                  # Launchers (RViz, controllers, state)
├─ docker/                  # Build/run scripts
└─ images/                  # README visuals
```

## 🧠 Nodes Overview

**G1Pilot** provides multiple ROS2 nodes to control and monitor the Unitree G1 robot.  

### 1️⃣ Cartesian Controller (`cartesian_controller`)  
- 🎯 **Purpose:** Controls the **right arm end-effector** of the robot.  
- 🔄 **Behavior:** Sends high-level Cartesian commands that are converted into joint movements via the **Joint Controller**.  
- 🛠️ **Use case:** Ideal for precise end-effector manipulation in Cartesian space.  

### 2️⃣ Joint Controller (`joint_controller`)  
- 🎯 **Purpose:** Directly controls the robot’s individual joints.  
- 🔄 **Behavior:** Receives low-level joint commands and sends them to the robot hardware for execution.  
- 🛠️ **Use case:** Required for fine-grained motion control and as a backend for Cartesian control.  

### 3️⃣ Interactive Marker (`interactive_marker`)  
- 🎯 **Purpose:** Publishes a **movable marker in RViz** to interactively control the robot’s end-effector.  
- 🛠️ **Use case:** Intuitive GUI control of the robot without manual command-line inputs.  

### 4️⃣ Robot State (`robot_state`)  
- 🎯 **Purpose:** Publishes the **complete state of the robot**, including:  
  - IMU readings  
  - Odometry  
  - Detailed motor states (temperature, voltage, position, velocity)  
- 🛠️ **Use case:** Used for monitoring and visualization in RViz or other tools.  

### 5️⃣ Loco Client (`loco_client`)  
- 🎯 **Purpose:** Enables **whole-body control and locomotion**, allowing the robot to move using **Unitree’s built-in policy**.  
- 🛠️ **Use case:** Autonomous or manual locomotion commands for walking and navigation.  

### 6️⃣ Joystick (`joystick`)  
- 🎯 **Purpose:** Integrates a **game controller (joystick)** to manually control the robot.  
- 🛠️ **Use case:** Remote teleoperation for research or demonstrations.  

#### **Basic Controls**
- **L1** → Emergency Stop → The robot enters **Damp Mode** (safe state).
- **⬆️ D-Pad Up** → Switches the robot to **FSM 4**, making it ready to receive commands.
- **R1** → Activates **Balance Mode**.
- **Left Joystick** → Controls **linear movements** (forward, backward, sideways).
- **Right Joystick** → Controls **angular rotation** (turning).

## Usage
Once you have the docker image running, you can run the following command to start the unitree node:

```bash
colcon build --symlink-install --packages-select g1pilot g1pilot
````

Then, source the workspace:

```bash
source install/setup.bash
```
To visualize the real robot in RViz, you can run the following command:

```bash
ros2 launch g1pilot robot_state_launcher.launch.py
```

To control the robot, using the joint controller or the cartesian controller, you can run the following command:
```bash
ros2 launch g1pilot controller_launcher.launch.py
```

To teleoperate the robot using the joystick, you can run the following command:
```bash
ros2 launch g1pilot teleoperation_launcher.launch.py
```

To apply autonomous navigation, you can run the following command:
```bash
ros2 launch g1pilot navigation_launcher.launch.py
```

## Contributing
We welcome contributions to **G1Pilot**! If you have suggestions, improvements, or bug fixes, please follow these steps:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Make your changes and commit them with clear messages.
4. Submit a pull request detailing your changes.

## License
BSD‑3‑Clause. See [LICENSE](LICENSE) for details.