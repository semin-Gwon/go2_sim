<div align="center">
  <h1>Go2 Intelligence Framework</h1>
  <p>ROS 2 & Isaac Sim based intelligence framework for Unitree Go2.</p>
</div>

## 🗺️ Project Roadmap
This project aims to build a comprehensive intelligence framework for the Unitree Go2 robot. We are planning to expand the framework step-by-step.

- [x] **3D SLAM (RTAB-Map)**: Visual and Depth based mapping in Isaac Sim.
- [x] **Navigation (Nav2)**: Autonomous path planning and obstacle avoidance.
- [ ] **Reinforcement Learning**: Advanced locomotion and task-specific policy training.
- [ ] **Real-world Deployment**: Sim2Real transfer and deployment on physical Go2 hardware.

---

## 📁 Repository Structure

```text
.
├── README.md                         # Project overview and run guide
├── assets/
│   └── envs/                         # Lightweight Isaac Sim USD scene files
│       ├── color_room.usd
│       ├── empty_room.usd
│       ├── object_room.usd
│       ├── room_env.usd
│       ├── slam_env.usd
│       └── yolo_env.usd
├── config/
│   ├── go2_nav2_params.yaml          # Nav2 parameters
│   └── go2_sim.rviz                  # RViz visualization config
├── docs/                             # Development plans and notes
│   ├── 00_archive_orb_slam3_task.md
│   ├── 01_rtabmap_slam_plan.md
│   ├── 01_rtabmap_slam_presentation.md
│   ├── 02_policy_decision.md
│   ├── 03_nav2_plan.md
│   └── 04_real_robot_deploy.md
├── launch/
│   ├── go2_navigation.launch.py      # Nav2 launch file
│   └── go2_rtabmap.launch.py         # RTAB-Map launch file
├── scripts/
│   ├── go2_sim.py                    # Main Isaac Sim + ROS2 simulation entry point
│   ├── my_slam_env.py                # Isaac Lab environment configuration
│   ├── cli_args.py                   # RSL-RL argument helpers
│   ├── deploy_scene_mcp.py           # Runtime scene deployment helper
│   ├── apt_30p_basic_furnished.py    # Apartment scene generation script
│   └── apt_4bay_furnished.py         # Apartment scene generation script
└── .gitignore                        # Local and generated file exclusions
```

The repository intentionally excludes generated outputs, large local assets, checkpoints, maps, and local assistant configuration files.

- `assets/objects/`
- `.pretrained_checkpoints/`
- `maps/`
- `outputs/`
- `scripts/outputs/`
- `__pycache__/`
- `CLAUDE.md`
- `.agent/`, `.agents/`, `.codex/`

## 🏗️ Modules

### 1. 3D SLAM (RTAB-Map in Isaac Sim)
Demonstrates 3D environmental mapping using RTAB-Map with the Go2 robot within the NVIDIA Isaac Sim environment.

#### 🎥 Demonstration Video
<div align="center">
  <a href="https://youtu.be/ZbYe7EWJfB8">
    <img src="https://img.youtube.com/vi/ZbYe7EWJfB8/0.jpg" alt="RTAB-Map SLAM Demonstration" width="600">
  </a>
  <p><i>Click the image to watch the RTAB-Map SLAM demonstration in action.</i></p>
</div>

#### 💻 Quick Start
To run the full simulation and SLAM pipeline, please open three separate terminals.

**Terminal A**: Start the Go2 simulation environment
```bash
cd ~/go2_sim
python scripts/go2_sim.py
```

**Terminal B**: Launch the RTAB-Map node
- **Mapping Mode** (for creating a new map):
```bash
cd ~/go2_sim
ros2 launch launch/go2_rtabmap.launch.py
```
- **Localization Mode** (Use this mode to estimate current position based on an existing map without creating a new one):
```bash
cd ~/go2_sim
ros2 launch launch/go2_rtabmap.launch.py localization:=true
```

**Terminal C**: Open RViz Visualization
```bash
cd ~/go2_sim
rviz2 -d config/go2_sim.rviz
```
> 💡 **Tip**: In Localization Mode, successful localization is confirmed when the red laser scan lines perfectly align with the generated map in RViz.

---

### 2. Autonomous Navigation (Nav2)
Integration with ROS 2 Nav2 stack for autonomous waypoint navigation and obstacle avoidance within the mapped environment.

#### 🎥 Demonstration Video
<div align="center">
  <a href="https://youtu.be/J8-3K4dXg9A">
    <img src="https://img.youtube.com/vi/J8-3K4dXg9A/0.jpg" alt="Nav2 Autonomous Navigation Demonstration" width="600">
  </a>
  <p><i>Click the image to watch the Nav2 demonstration in action.</i></p>
</div>

#### 💻 Quick Start
To run the Nav2 autonomous navigation, follow these steps in separate terminals.

**Terminal A**: Start the Go2 simulation environment
```bash
cd ~/go2_sim
python scripts/go2_sim.py
```

**Terminal B**: Launch the Nav2 stack
```bash
cd ~/go2_sim
ros2 launch launch/go2_navigation.launch.py
```

**Terminal C**: Open RViz Visualization
```bash
cd ~/go2_sim
rviz2 -d config/go2_sim.rviz
```
> ⚠️ **Note**: Please ensure to issue the `2D Goal Pose` from RViz **only after** the robot's localization is successfully completed.
---

### 3. Reinforcement Learning
*Coming Soon: RL environment setup and policy training for Go2 locomotion and intelligent behavior.*

---

### 4. Real-world Deployment
*Coming Soon: Guidelines and scripts for deploying the developed intelligence on the actual Unitree Go2 robot.*

---

## ⚙️ Local Path Notes

Some scripts still contain machine-specific paths used during development, such as Isaac Lab, local object assets, policy checkpoints, and optional perception config files.

Before running this project on another machine, check and update these paths or provide equivalent environment variables:

- Isaac Sim / Isaac Lab Python site-packages path in `scripts/go2_sim.py`
- `GO2_SIM_VISIBLE_OBJECTS_CONFIG`
- `GO2_SIM_NAMED_PLACES_CONFIG`
- `GO2_SIM_DYNAMIC_PERSON_USD`
- `GO2_SIM_VISUAL_ENV_USD`
- local policy checkpoint directory used by the RL runner

Large local assets and generated files are intentionally excluded from GitHub:

- `assets/objects/`
- `.pretrained_checkpoints/`
- `maps/`
- `outputs/`
- `scripts/outputs/`
