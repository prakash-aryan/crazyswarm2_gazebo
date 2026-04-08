# Crazyswarm2 Gazebo Simulation

Crazyflie swarm simulation using **Crazyswarm2** + **Gazebo Harmonic** on **ROS 2 Jazzy**.

Write your swarm scripts once using the Crazyswarm2 Python API (`takeoff`, `land`, `goTo`, trajectories) and run them in Gazebo simulation. The architecture is designed so the same scripts can later run on real Crazyflie hardware by switching the backend.

## Architecture

![Architecture](docs/media/architecture.png)

Crazyswarm2 server runs the actual Crazyflie firmware (compiled to Python bindings) in a software-in-the-loop setup. A PD velocity controller converts desired states to `cmd_vel` commands, which flow through the ROS-Gazebo bridge to Gazebo Harmonic's multirotor physics engine. Odometry flows back the same way.

## Demos

### Swarm Demo
3 drones take off, form a circle, rotate through each other's positions, return to start, and land.

![Swarm Demo](docs/media/swarm_demo.gif)

### Figure-8 Trajectory
All 3 drones execute a figure-8 piecewise polynomial trajectory uploaded from CSV.

![Figure 8](docs/media/figure8.gif)

### Multi-Trajectory
Each drone flies a different precomputed trajectory simultaneously.

![Multi Trajectory](docs/media/multi_trajectory.gif)

## Prerequisites

- **Ubuntu 24.04**
- **ROS 2 Jazzy** (desktop install)
- **Gazebo Harmonic** (gz sim 8)

Install ROS 2 and Gazebo:
```bash
# ROS 2 Jazzy
sudo apt install ros-jazzy-desktop ros-dev-tools

# Gazebo bridge
sudo apt install ros-jazzy-ros-gz

# Additional ROS packages
sudo apt install ros-jazzy-motion-capture-tracking ros-jazzy-tf-transformations
```

Install Python dependencies:
```bash
pip install rowan nicegui cflib transforms3d
```

## Setup

### 1. Clone this repo with submodules

```bash
git clone --recursive https://github.com/prakash-aryan/crazyswarm2_gazebo.git
cd crazyswarm2_gazebo
# Initialize nested submodules (crazyflie_tools, etc.)
cd src/crazyswarm2 && git submodule update --init --recursive && cd ../..
```

### 2. Build the Crazyflie firmware Python bindings

```bash
cd ~
git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git
cd crazyflie-firmware
make cf2_defconfig
make bindings_python
```

### 3. Build the workspace

```bash
cd ~/crazyswarm2_gazebo
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> **Important — Python version mismatch:** ROS 2 Jazzy requires **Python 3.12** (the system Python on Ubuntu 24.04). If you have a different Python version active (e.g. Python 3.14 via mise/pyenv), the build may succeed but packages will **fail at runtime** with missing module errors because the compiled `.so` bindings (e.g. from crazyflie-firmware) won't match.
>
> **Fix:** Disable your version manager and force the system Python before building **and** running:
> ```bash
> export MISE_DISABLED=1
> export PATH="/usr/bin:$PATH"
> python3 --version  # should show 3.12.x
> ```
> If you already built with the wrong Python, clean and rebuild:
> ```bash
> rm -rf build/ install/ log/
> colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
> ```

## Running the Simulation

### Terminal 1: Launch Gazebo + Crazyswarm2 + RViz

```bash
source /opt/ros/jazzy/setup.bash
source ~/crazyswarm2_gazebo/install/local_setup.bash
export PYTHONPATH="$HOME/crazyflie-firmware/build:$PYTHONPATH"
export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix ros_gz_crazyflie_gazebo)/share/ros_gz_crazyflie_gazebo/models:$GZ_SIM_RESOURCE_PATH"

ros2 launch ros_gz_crazyflie_bringup gazebo_crazyswarm2.launch.py
```

This starts:
- Gazebo Harmonic with 3 Crazyflie drones (cf1, cf3, cf4)
- ROS-Gazebo bridge (cmd_vel, odometry, clock)
- Crazyswarm2 server with the Gazebo backend
- Consolidated TF publisher
- RViz with robot models

### Terminal 2: Run a demo script

```bash
source /opt/ros/jazzy/setup.bash
source ~/crazyswarm2_gazebo/install/local_setup.bash
export PYTHONPATH="$HOME/crazyflie-firmware/build:$PYTHONPATH"
```

All demos use the Crazyswarm2 Python API and require `use_sim_time`:

```bash
ros2 run crazyflie_examples <demo_name> --ros-args -p use_sim_time:=True
```

### Available demos

| Demo | Drones | Description |
|---|---|---|
| `hello_world` | 1 (cf1) | Single drone takeoff, hover 5s, land |
| `swarm_demo` | All 3 | Circle formation, rotate positions, return to start, land |
| `figure8` | All 3 | Upload and execute figure-8 polynomial trajectory from CSV |
| `multi_trajectory` | All 3 | Each drone flies a different trajectory |
| `cmd_full_state` | 1 (cf1) | Stream full-state setpoints (pos/vel/acc/yaw/omega) at 30 Hz |
| `group_mask` | All 3 | Selectively command subsets of the swarm |

> **Note:** `swap` requires drone IDs 231/5 which are not configured in this setup. `nice_hover` requires an interactive button press. `infinite_flight` and `arming` are for real hardware only.

## Writing Your Own Swarm Scripts

Use the standard Crazyswarm2 Python API:

```python
from crazyflie_py import Crazyswarm

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # Takeoff all drones
    allcfs.takeoff(targetHeight=1.0, duration=3.0)
    timeHelper.sleep(4.0)

    # Command individual drones
    for cf in allcfs.crazyflies:
        cf.goTo([0.0, 0.0, 1.0], 0, 3.0)
    timeHelper.sleep(4.0)

    # Land
    allcfs.land(targetHeight=0.04, duration=3.0)
    timeHelper.sleep(4.0)

if __name__ == '__main__':
    main()
```

## Configuration

### Drone positions

Edit `src/crazyswarm2/crazyflie/config/crazyflies.yaml` to change drone count and positions. Ensure positions match the `<pose>` values in `src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/worlds/crazyflie_swarm_world.sdf`.

### Gazebo backend tuning

The PD gains in `src/crazyswarm2/crazyflie_sim/crazyflie_sim/backend/gazebo.py`:
```python
self.kp_pos = np.array([0.8, 0.8, 1.0])   # Position proportional gain
self.kd_vel = np.array([0.5, 0.5, 0.5])   # Velocity damping
self.kff_vel = np.array([0.3, 0.3, 0.3])  # Velocity feedforward
self.max_vel = 1.0                          # Max velocity clamp (m/s)
```

## What We Built

- **Gazebo backend** for Crazyswarm2 — a new `gazebo.py` backend that bridges the SIL firmware to Gazebo Harmonic via velocity commands and odometry
- **3-drone world** SDF with Crazyflie multirotor physics (motor models, velocity controllers)
- **ROS-Gazebo bridge** config for cmd_vel, odometry, enable, and clock topics
- **Consolidated TF publisher** to avoid time-jump issues with multiple odometry sources
- **Swarm demo script** showing coordinated multi-drone flight
- **PD controller tuning** with velocity clamping for stable trajectory tracking

## Roadmap

- [ ] Test with real Crazyflie hardware (switch backend to `cflib` or `cpp`)
- [ ] Obstacle avoidance with multi-ranger deck, Flow deck, and Nav2 ([crazyflie_ros2_multiranger](https://github.com/knmcguire/crazyflie_ros2_multiranger))
- [ ] Leader-follower formation flying
- [ ] Scale to 10+ drones
- [ ] Integration with motion planning (db-CBS trajectory planner)

## Project Structure

```
crazyswarm2_gazebo/
├── src/
│   ├── crazyswarm2/                    # Fork of IMRCLab/crazyswarm2
│   │   ├── crazyflie/config/
│   │   │   ├── crazyflies.yaml         # Drone definitions (3 drones)
│   │   │   └── server.yaml             # Backend set to 'gazebo'
│   │   ├── crazyflie_sim/backend/
│   │   │   └── gazebo.py               # Gazebo backend (NEW)
│   │   └── crazyflie_examples/
│   │       └── swarm_demo.py           # Swarm demo script (NEW)
│   └── ros_gz_crazyflie/               # Fork of knmcguire/ros_gz_crazyflie
│       ├── ros_gz_crazyflie_bringup/
│       │   ├── config/
│       │   │   ├── ros_gz_crazyflie_swarm_bridge.yaml  # Bridge config (NEW)
│       │   │   └── swarm_rviz.rviz                     # RViz config (NEW)
│       │   └── launch/
│       │       └── gazebo_crazyswarm2.launch.py        # Main launch (NEW)
│       ├── ros_gz_crazyflie_control/
│       │   └── odom_tf_publisher.py    # Consolidated TF publisher (NEW)
│       └── ros_gz_crazyflie_gazebo/
│           ├── models/crazyflie/
│           │   └── crazyflie.urdf      # RViz model (NEW)
│           └── worlds/
│               └── crazyflie_swarm_world.sdf  # 3-drone world (NEW)
├── docs/media/                         # Demo GIFs and architecture diagram
└── README.md
```

## Acknowledgments

Built on top of:
- [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2) by IMRCLab
- [ros_gz_crazyflie](https://github.com/knmcguire/ros_gz_crazyflie) by Kimberly McGuire / Bitcraze
- [Crazyflie firmware](https://github.com/bitcraze/crazyflie-firmware) by Bitcraze
