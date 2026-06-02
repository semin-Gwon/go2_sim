# Go2 Intelligence Framework

> This repository is based on content from [ctrlcvlab/Go2_Intelligence_Framework](https://github.com/ctrlcvlab/Go2_Intelligence_Framework).  
> It has been reorganized and adapted for this local `go2_sim` workspace and portfolio-oriented GitHub publication.

`go2_sim` is a ROS2 and Isaac Sim based simulation workspace for Unitree Go2. The project focuses on running Go2 in Isaac Sim, publishing simulated camera/odometry/IMU data through ROS2, building a map with RTAB-Map, and running autonomous navigation with Nav2.

## Main Features

- Unitree Go2 simulation entry point using Isaac Sim / Isaac Lab.
- RGB-D camera, odometry, IMU, TF, and simulation clock publishing through ROS2 bridge.
- RTAB-Map based 3D SLAM and localization workflow.
- Nav2 based autonomous navigation using the RTAB-Map localization output.
- RViz configuration for monitoring SLAM, localization, and navigation.
- Lightweight USD scene files for simulation environments.

## Repository Structure

```text
.
├── README.md
├── assets/
│   └── envs/
│       ├── color_room.usd
│       ├── empty_room.usd
│       ├── object_room.usd
│       ├── room_env.usd
│       ├── slam_env.usd
│       └── yolo_env.usd
├── config/
│   ├── go2_nav2_params.yaml
│   └── go2_sim.rviz
├── docs/
│   ├── 00_archive_orb_slam3_task.md
│   ├── 01_rtabmap_slam_plan.md
│   ├── 01_rtabmap_slam_presentation.md
│   ├── 02_policy_decision.md
│   ├── 03_nav2_plan.md
│   └── 04_real_robot_deploy.md
├── launch/
│   ├── go2_navigation.launch.py
│   └── go2_rtabmap.launch.py
├── scripts/
│   ├── go2_sim.py
│   ├── my_slam_env.py
│   ├── cli_args.py
│   ├── deploy_scene_mcp.py
│   ├── apt_30p_basic_furnished.py
│   └── apt_4bay_furnished.py
└── .gitignore
```

## Key Files

| Path | Role |
| --- | --- |
| `scripts/go2_sim.py` | Main Isaac Sim execution script. Launches the Go2 simulation, enables cameras, loads the policy runner, and publishes ROS2 data through the Isaac Sim ROS2 bridge. |
| `scripts/my_slam_env.py` | Isaac Lab environment configuration. Selects the visual USD environment and configures sensors. |
| `launch/go2_rtabmap.launch.py` | RTAB-Map SLAM/localization launch file. Uses paths relative to this repository and stores generated map databases under `maps/`. |
| `launch/go2_navigation.launch.py` | Nav2 launch file. Starts RTAB-Map in localization mode and includes Nav2 navigation launch with this repository's config. |
| `config/go2_nav2_params.yaml` | Nav2 parameter file. |
| `config/go2_sim.rviz` | RViz visualization configuration. |
| `assets/envs/*.usd` | Lightweight Isaac Sim scene files included in GitHub. |

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- NVIDIA Isaac Sim / Isaac Lab environment
- Python environment compatible with Isaac Lab
- RTAB-Map ROS packages
- Nav2 packages
- RViz2

The simulation script currently contains machine-specific Isaac Sim / Isaac Lab paths. Before running on a different PC, update those paths in `scripts/go2_sim.py` or provide equivalent local environment variables.

## Run SLAM

Run the SLAM pipeline from this folder. Use three terminals.

### Terminal A: Start Go2 Simulation

```bash
cd ~/go2_sim
python scripts/go2_sim.py
```

For faster headless-style simulation without real-time rendering:

```bash
cd ~/go2_sim
python scripts/go2_sim.py --rt false
```

### Terminal B: Start RTAB-Map SLAM

Mapping mode creates a new RTAB-Map database under `maps/`.

```bash
cd ~/go2_sim
source /opt/ros/humble/setup.bash
ros2 launch launch/go2_rtabmap.launch.py
```

### Terminal C: Open RViz

```bash
cd ~/go2_sim
source /opt/ros/humble/setup.bash
rviz2 -d config/go2_sim.rviz
```

Generated map databases are intentionally excluded from GitHub:

```text
maps/
*.db
*.db3
```

## Run Localization

After a map database has been created locally, RTAB-Map can run in localization mode.

```bash
cd ~/go2_sim
source /opt/ros/humble/setup.bash
ros2 launch launch/go2_rtabmap.launch.py localization:=true
```

This mode loads the existing database from:

```text
maps/rtabmap_ground_truth.db
```

## Run Navigation

Navigation uses RTAB-Map localization and Nav2 together. Create or provide a local RTAB-Map database first, then use three terminals.

### Terminal A: Start Go2 Simulation

```bash
cd ~/go2_sim
python scripts/go2_sim.py
```

### Terminal B: Start Navigation

`go2_navigation.launch.py` starts RTAB-Map in localization mode and then starts Nav2 with `config/go2_nav2_params.yaml`.

```bash
cd ~/go2_sim
source /opt/ros/humble/setup.bash
ros2 launch launch/go2_navigation.launch.py
```

### Terminal C: Open RViz

```bash
cd ~/go2_sim
source /opt/ros/humble/setup.bash
rviz2 -d config/go2_sim.rviz
```

In RViz, send a `2D Goal Pose` only after localization is stable and the scan/map alignment looks correct.

## ROS2 Topics Used by the Pipeline

The simulation and launch files are organized around these topics:

| Topic | Purpose |
| --- | --- |
| `/camera/color/image_raw` | RGB camera image |
| `/camera/depth/image_rect_raw` | Depth image |
| `/camera/camera_info` | Camera calibration info |
| `/odom` | Robot odometry |
| `/imu/data` | IMU data |
| `/tf` | TF tree |
| `/clock` | Simulation time |
| `/scan` | Laser scan generated from depth image |
| `/map` | RTAB-Map map output |

## Local Path Notes

The following paths or environment variables may need adjustment on another machine:

- Isaac Sim / Isaac Lab site-packages path in `scripts/go2_sim.py`
- `GO2_SIM_VISIBLE_OBJECTS_CONFIG`
- `GO2_SIM_NAMED_PLACES_CONFIG`
- `GO2_SIM_DYNAMIC_PERSON_USD`
- `GO2_SIM_VISUAL_ENV_USD`
- local RSL-RL / policy checkpoint path
- local object asset directory under `assets/objects/`

Large or machine-local files are excluded from GitHub:

```text
assets/objects/
.pretrained_checkpoints/
maps/
outputs/
scripts/outputs/
__pycache__/
CLAUDE.md
.agent/
.agents/
.codex/
```

## Notes on Assets and Checkpoints

Only lightweight USD environment files under `assets/envs/` are tracked. Large object assets, generated maps, local checkpoints, and local assistant configuration files are intentionally omitted.

If a script references an omitted asset, place the required file in the expected local path or update the corresponding environment variable before running.

## License and Attribution

This repository is adapted from [ctrlcvlab/Go2_Intelligence_Framework](https://github.com/ctrlcvlab/Go2_Intelligence_Framework). Check the upstream repository and any third-party packages or assets for their original licenses.

Custom changes in this local repository do not currently define a separate root license. Before redistribution beyond personal portfolio use, review upstream licenses and add an explicit root `LICENSE` file if needed.
