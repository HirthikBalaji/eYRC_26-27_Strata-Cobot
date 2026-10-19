# e-Yantra Robotics Competition 2026-27 — StrataCobot (SC)

> **Theme**: StrataCobot (Kepler Colony)  
> **Environment**: Ubuntu 24.04 LTS · ROS 2 Jazzy Jalisco · Gazebo Harmonic (gz-sim 8) · Python 3.12

---

## 📖 Overview

The **StrataCobot** challenge simulates autonomous resource processing in an extraterrestrial Martian colony. In **Task 2A**, a UR7e robotic arm is deployed at the ore processing cell to detect raw ores on a feeder conveyor, grasp them with an end-effector magnetic tool, and sort them into their designated cargo rover storage containers without triggering protective stops or collisions.

![Cell Layout](https://portal.e-yantra.org/courses/theme_sc/stage_1/task_2/task_2a/images/sorting_containers.jpg)

### Ore Kinds & Container Destinations

| Ore Type | Mineral | Color | Cargo Rover Container | Coordinates (`base_link`) |
| :--- | :--- | :--- | :--- | :--- |
| `azurite_ore` | Azurite | Deep Blue | **Farthest** container | `[-0.7430, 0.5280, 0.2300]` |
| `malachite_ore` | Malachite | Vibrant Green | **Middle** container | `[-0.4097, 0.5280, 0.2300]` |
| `vanadinite_ore` | Vanadinite | Red-Orange | **Nearest** container | `[-0.0763, 0.5280, 0.2300]` |

---

## 🛠 What Was Created & Configured

### 1. `task2A_perception.py`
- Subscribes to the camera topics: `/camera/camera/color/image_raw`, `/camera/camera/aligned_depth_to_color/image_raw`, and `/camera/camera/color/camera_info`.
- Uses HSV color segmentation and morphological filtering for the three ore types:
  - **Azurite** (blue)
  - **Malachite** (green)
  - **Vanadinite** (orange/red)
- Samples depth at the centroid of each ore's top face and deprojects pixels into 3D camera coordinates:
  $$X_c = \frac{(cX - c_x) \cdot Z}{f_x}, \quad Y_c = \frac{(cY - c_y) \cdot Z}{f_y}, \quad Z_c = Z$$
- Transforms positions to `base_link` and filters detections bounded within the conveyor volume ($X \in [-0.90, 0.35], Y \in [-0.75, -0.30], Z \in [0.10, 0.30]$).
- Implements persistent ID tracking (`<kind>_1`, `<kind>_2`) and continuously broadcasts `TransformStamped` frames on `/tf`.
- Naturally stops publishing when an ore is picked up and lifted off the conveyor.

### 2. `task2A_manipulation.py`
- Coordinates with `delta_twist_controller` via `/delta_twist_cmds` at 20 Hz, safely below the velocity caps (0.10 m/s linear, 0.20 rad/s angular).
- Keeps the magnetic tool oriented strictly vertically downward towards the conveyor and containers at all times using axis-angle alignment:
  $$\vec{c} = \vec{a} \times \vec{b}, \quad \theta = \text{atan2}(\|\vec{c}\|, \vec{a} \cdot \vec{b}), \quad \vec{\omega} = K_{rot} \cdot \theta \cdot \frac{\vec{c}}{\|\vec{c}\|}$$
- Queries `/tf` for detected ores and routes each ore to its designated container on the cargo rover:
  - **Azurite**: Farthest container (`x = -0.7430, y = 0.5280, z = 0.2300`)
  - **Malachite**: Middle container (`x = -0.4097, y = 0.5280, z = 0.2300`)
  - **Vanadinite**: Nearest container (`x = -0.0763, y = 0.5280, z = 0.2300`)
- Uses a safe transit waypoint (`[-0.8000, 0.0000, 0.4000]`) between the conveyor and the cargo rover to avoid singularities and base collisions.
- Monitors force feedback on `/ur7e/end_effector/force_magnitude` and coordinates `/magnet` engagement/release.
- Listens to `/arm_status` and triggers `/ur_arm_controller/unlock_protective_stop` if a protective stop occurs.

### 3. `setup.py`
- Registered both scripts in the `SCRIPTS` array of `algorithms/setup.py`:
  ```python
  SCRIPTS = [
      'scripts/task2a/task2A_perception.py',
      'scripts/task2a/task2A_manipulation.py',
  ]
  ```

---

## 🚀 Running & Evaluating Task 2A

In your Ubuntu 24.04 environment (with ROS 2 Jazzy and Gazebo Harmonic sourced):

### 1. Build and install the nodes
```bash
cd ~/ros2_ws   # or the path containing eYRC_26-27_Strata-Cobot
colcon build --packages-select algorithms
source install/setup.bash
ros2 pkg executables algorithms
```

### 2. Terminal 1 — Start the simulation world
```bash
ros2 launch eyantra_kepler_colony task2a.launch.py
```
Wait until Gazebo and RViz open and print:
```text
[launch.user]: Task 2A world ready: Start your nodes now.
```

### 3. Terminal 2 — Run the evaluator (for scoring/submission)
```bash
./eyrc-sc-evaluator --task 2A --team-id <YOUR_TEAM_ID>
```
Wait until the evaluator prints `Ready`.

### 4. Terminals 3 & 4 — Run your nodes
In Terminal 3:
```bash
ros2 run algorithms task2A_perception.py
```

In Terminal 4:
```bash
ros2 run algorithms task2A_manipulation.py
```

### 5. Complete & Save Run
Once all 4 ores have been picked, placed, and have settled in their respective containers, press `q` in the evaluator terminal. The evaluator will generate `result.zip` for submission.

---

## 📂 Repository Structure

```text
eYRC_26-27_Strata-Cobot/
├── algorithms/
│   ├── boilerplate/                  # Task boilerplate templates
│   ├── scripts/
│   │   └── task2a/
│   │       ├── task2A_perception.py  # Ore detection & tf2 broadcaster
│   │       └── task2A_manipulation.py# Arm Cartesian trajectory & magnet controller
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py                      # Installed node definitions
├── eyantra_kepler_colony/            # Gazebo Kepler arena worlds, models & launch files
├── ur_description/                   # UR7e arm model, meshes, controllers & plugins
├── ebot_description/                 # eBot mobile platform assets
├── eyrc-sc-evaluator                 # Official task evaluation tool
├── requirements.sh                   # System dependencies check & install script
└── README.md
```

---

## 🔍 Verification & Monitoring Topics

To monitor published ore transforms:
```bash
ros2 run tf2_ros tf2_echo base_link azurite_ore_1
```

To monitor end-effector contact force:
```bash
ros2 topic echo /ur7e/end_effector/force_magnitude
```

To monitor arm safety status:
```bash
ros2 topic echo /arm_status
```
