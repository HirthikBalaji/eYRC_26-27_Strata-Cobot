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

## 🏗 System Architecture & Implementation

### 1. Perception System (`task2A_perception.py`)
- **Camera Streams**: Subscribes to the overhead Intel RealSense RGB-D camera:
  - `/camera/camera/color/image_raw` (640×480 BGR colour)
  - `/camera/camera/aligned_depth_to_color/image_raw` (aligned 32FC1 depth in metres)
  - `/camera/camera/color/camera_info` (camera intrinsic matrix)
- **HSV Color Segmentation**: Converts incoming BGR frames to HSV space, filters each ore with tuned color bounds, and applies morphological opening and closing to suppress noise.
- **Centroid & Depth Sampling**: Calculates spatial moments $(cX, cY)$ for detected contours and extracts the median depth across a $9 \times 9$ pixel patch over the ore's flat top surface.
- **Pinhole Deprojection**:
  $$X_c = \frac{(cX - c_x) \cdot Z}{f_x}, \quad Y_c = \frac{(cY - c_y) \cdot Z}{f_y}, \quad Z_c = Z$$
- **Frame Transformation**: Transforms coordinates from `camera_color_optical_frame` into the arm's reference frame `base_link` using tf2 buffer lookups.
- **Conveyor Region Filtering**: Filters out spurious detections outside the conveyor bounding envelope ($X \in [-0.90, 0.35], Y \in [-0.75, -0.30], Z \in [0.10, 0.30]$).
- **Persistent Tracking & TF Broadcasting**: Continuously publishes tf2 transforms (`<ore_type>_<id>`, e.g., `azurite_ore_1`, `malachite_ore_1`) with `base_link` as the parent frame. Unmatched detections get sequentially assigned IDs. Broadcast automatically stops once an ore is picked up and lifted off the conveyor.

### 2. Arm Manipulation & Control (`task2A_manipulation.py`)
- **Cartesian Velocity Servo**: Publishes `geometry_msgs/msg/TwistStamped` on `/delta_twist_cmds` at 20 Hz, safely below hardware caps ($v_{max} = 0.10\text{ m/s} < 0.15\text{ m/s}$, $\omega_{max} = 0.20\text{ rad/s} < 0.35\text{ rad/s}$).
- **Active Controller Management**: Interacts with `/ur_arm_controller/switch_controller` to switch between `delta_joint_controller` and `delta_twist_controller`.
- **Tool Orientation Alignment**: Enforces the magnetic tool axis ($Z$-axis of TCP) to point vertically downward ($[0, 0, -1]$ in `base_link`) using axis-angle proportional rotation control:
  $$\vec{c} = \vec{a} \times \vec{b}, \quad \theta = \text{atan2}(\|\vec{c}\|, \vec{a} \cdot \vec{b}), \quad \vec{\omega} = K_{rot} \cdot \theta \cdot \frac{\vec{c}}{\|\vec{c}\|}$$
- **State Machine Workflow**:
  1. `INIT_WAIT`: Ensures controller activation, initializes magnet to OFF, and waits for stable ore detection.
  2. `FIND_NEXT_ORE`: Discovers active ore transforms via TF and plans the next target.
  3. `MOVE_HOVER_ORE`: Approaches pre-pick waypoint ($Z = 0.35\text{ m}$).
  4. `DESCEND_PICK`: Descents gently ($v \le 0.035\text{ m/s}$) until contact is confirmed via force sensor (`/ur7e/end_effector/force_magnitude` $> 1.0\text{ N}$) or vertical threshold.
  5. `WAIT_MAGNET`: Calls `/magnet` service (`data=True`) and confirms grip.
  6. `LIFT_ORE`: Ascends vertically to $Z = 0.38\text{ m}$ off the conveyor.
  7. `TRANSIT_CONTAINER`: Traverses via intermediate safety corridor waypoint `[-0.8000, 0.0000, 0.4000]` to avoid arm self-collision or base collision.
  8. `MOVE_HOVER_CONTAINER`: Positions over the matching container ($Z = 0.35\text{ m}$).
  9. `DESCEND_CONTAINER`: Lowers to release height ($Z = 0.23\text{ m}$).
  10. `RELEASE_ORE`: Calls `/magnet` (`data=False`), waits $0.8\text{ s}$ for ore to settle inside container, and marks ore as completed.
  11. `ASCEND_CONTAINER`: Ascends safely back to $Z = 0.38\text{ m}$.
  12. `TRANSIT_CONVEYOR`: Returns via transit waypoint to repeat for all 4 ores.
  13. `ALL_DONE`: Parks arm safely and holds pose.
- **Protective Stop Recovery**: Listens to `/arm_status` codes $20$–$23$ and automatically issues `/ur_arm_controller/unlock_protective_stop` requests.

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

## 🚀 Setup & Execution Guide

### Prerequisites
- **Operating System**: Ubuntu 24.04 LTS
- **ROS 2 Distribution**: Jazzy Jalisco (`ros-jazzy-desktop-full`)
- **Simulator**: Gazebo Harmonic (`gz-sim 8`)

### 1. Build the Workspace
From the workspace root directory:
```bash
./requirements.sh
cd ..
colcon build --packages-select algorithms
source install/setup.bash
```

Confirm both nodes are registered:
```bash
ros2 pkg executables algorithms
```
*Expected output includes `task2A_perception.py` and `task2A_manipulation.py`.*

---

### 2. Running Task 2A

Use separate terminal tabs for each process (ensuring each is sourced with `source install/setup.bash`):

#### Terminal 1 — Start the Kepler World
```bash
ros2 launch eyantra_kepler_colony task2a.launch.py
```
> Wait until Gazebo and RViz finish loading and print:  
> `[launch.user]: Task 2A world ready: Start your nodes now.`

#### Terminal 2 — Run the Evaluator
```bash
./eyrc-sc-evaluator --task 2A --team-id <YOUR_TEAM_ID>
```
> Wait until the evaluator prints `Ready`.

#### Terminal 3 — Run Perception
```bash
ros2 run algorithms task2A_perception.py
```

#### Terminal 4 — Run Arm Manipulation
```bash
ros2 run algorithms task2A_manipulation.py
```

---

### 3. Verification & Monitoring

To monitor published transforms:
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

---

### 4. Submission
When all four ores have settled in their respective containers:
1. Press `q` in the evaluator terminal.
2. The evaluator will generate `result.zip`.
3. Package `result.zip` with `task2A_perception.py` and `task2A_manipulation.py` as specified in the e-Yantra submission portal.
