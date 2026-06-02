# Unitree Go2 Isaac Sim SLAM & Navigation

언어: [English](README.md) | 한국어

> 이 저장소는 [leesj24601/Go2_Intelligence_Framework](https://github.com/leesj24601/Go2_Intelligence_Framework)의 내용을 기반으로 합니다.  
> 현재 로컬 `go2_sim` 워크스페이스와 포트폴리오용 GitHub 공개에 맞게 재정리하고 수정했습니다.

`go2_sim`은 Unitree Go2를 위한 ROS2 및 Isaac Sim 기반 시뮬레이션 워크스페이스입니다. Isaac Sim에서 Go2를 실행하고, 시뮬레이션 카메라/오도메트리/IMU 데이터를 ROS2로 publish하며, RTAB-Map으로 맵을 생성하고 Nav2로 자율 주행을 실행하는 흐름을 중심으로 구성되어 있습니다.

## 주요 기능

- Isaac Sim / Isaac Lab 기반 Unitree Go2 시뮬레이션 실행.
- ROS2 Bridge를 통한 RGB-D 카메라, odometry, IMU, TF, simulation clock publish.
- RTAB-Map 기반 3D SLAM 및 localization workflow.
- RTAB-Map localization 출력을 활용한 Nav2 자율 주행.
- SLAM, localization, navigation 상태 확인을 위한 RViz 설정.
- 시뮬레이션 환경용 경량 USD scene 파일 포함.

## 저장소 구조

```text
.
├── README.md
├── README.ko.md
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

## 주요 파일

| Path | 역할 |
| --- | --- |
| `scripts/go2_sim.py` | 메인 Isaac Sim 실행 스크립트입니다. Go2 시뮬레이션을 실행하고, camera를 활성화하며, policy runner를 로드하고 Isaac Sim ROS2 Bridge를 통해 ROS2 데이터를 publish합니다. |
| `scripts/my_slam_env.py` | Isaac Lab 환경 설정 파일입니다. visual USD 환경과 sensor 구성을 담당합니다. |
| `launch/go2_rtabmap.launch.py` | RTAB-Map SLAM/localization launch 파일입니다. 저장소 기준 상대 경로를 사용하며 생성된 map database를 `maps/` 아래에 저장합니다. |
| `launch/go2_navigation.launch.py` | Nav2 launch 파일입니다. RTAB-Map을 localization mode로 실행하고, 이 저장소의 Nav2 config를 사용해 navigation launch를 포함합니다. |
| `config/go2_nav2_params.yaml` | Nav2 parameter 파일입니다. |
| `config/go2_sim.rviz` | RViz 시각화 설정 파일입니다. |
| `assets/envs/*.usd` | GitHub에 포함한 경량 Isaac Sim scene 파일입니다. |

## 사전 준비

- Ubuntu 22.04
- ROS2 Humble
- NVIDIA Isaac Sim / Isaac Lab 환경
- Isaac Lab과 호환되는 Python 환경
- RTAB-Map ROS 패키지
- Nav2 패키지
- RViz2

현재 시뮬레이션 스크립트에는 개발 PC 기준 Isaac Sim / Isaac Lab 경로가 포함되어 있습니다. 다른 PC에서 실행하려면 `scripts/go2_sim.py`의 경로를 수정하거나 동일한 역할의 환경 변수를 제공해야 합니다.

## SLAM 실행

이 폴더 기준으로 SLAM pipeline을 실행합니다. 터미널 3개를 사용합니다.

### Terminal A: Go2 Simulation 실행

```bash
cd ~/go2_sim
python scripts/go2_sim.py
```

실시간 렌더링 없이 더 빠르게 실행하려면:

```bash
cd ~/go2_sim
python scripts/go2_sim.py --rt false
```

### Terminal B: RTAB-Map SLAM 실행

Mapping mode는 `maps/` 아래에 새 RTAB-Map database를 생성합니다.

```bash
cd ~/go2_sim
source /opt/ros/humble/setup.bash
ros2 launch launch/go2_rtabmap.launch.py
```

### Terminal C: RViz 실행

```bash
cd ~/go2_sim
source /opt/ros/humble/setup.bash
rviz2 -d config/go2_sim.rviz
```

생성된 map database는 GitHub에서 제외됩니다.

```text
maps/
*.db
*.db3
```

## Localization 실행

로컬에서 map database를 생성한 뒤에는 RTAB-Map을 localization mode로 실행할 수 있습니다.

```bash
cd ~/go2_sim
source /opt/ros/humble/setup.bash
ros2 launch launch/go2_rtabmap.launch.py localization:=true
```

이 mode는 다음 database를 로드합니다.

```text
maps/rtabmap_ground_truth.db
```

## Navigation 실행

Navigation은 RTAB-Map localization과 Nav2를 함께 사용합니다. 먼저 로컬 RTAB-Map database를 생성하거나 준비한 뒤 터미널 3개를 사용합니다.

### Terminal A: Go2 Simulation 실행

```bash
cd ~/go2_sim
python scripts/go2_sim.py
```

### Terminal B: Navigation 실행

`go2_navigation.launch.py`는 RTAB-Map을 localization mode로 실행한 뒤 `config/go2_nav2_params.yaml`을 사용해 Nav2를 실행합니다.

```bash
cd ~/go2_sim
source /opt/ros/humble/setup.bash
ros2 launch launch/go2_navigation.launch.py
```

### Terminal C: RViz 실행

```bash
cd ~/go2_sim
source /opt/ros/humble/setup.bash
rviz2 -d config/go2_sim.rviz
```

RViz에서는 localization이 안정화되고 scan/map alignment가 맞는 것을 확인한 뒤 `2D Goal Pose`를 지정합니다.

## ROS2 Topic

시뮬레이션과 launch 파일은 다음 topic을 기준으로 구성되어 있습니다.

| Topic | 목적 |
| --- | --- |
| `/camera/color/image_raw` | RGB camera image |
| `/camera/depth/image_rect_raw` | Depth image |
| `/camera/camera_info` | Camera calibration info |
| `/odom` | Robot odometry |
| `/imu/data` | IMU data |
| `/tf` | TF tree |
| `/clock` | Simulation time |
| `/scan` | Depth image에서 생성한 laser scan |
| `/map` | RTAB-Map map output |

## 로컬 경로 주의사항

다른 PC에서 실행할 때는 다음 경로나 환경 변수를 확인해야 합니다.

- `scripts/go2_sim.py` 내부 Isaac Sim / Isaac Lab site-packages 경로
- `GO2_SIM_VISIBLE_OBJECTS_CONFIG`
- `GO2_SIM_NAMED_PLACES_CONFIG`
- `GO2_SIM_DYNAMIC_PERSON_USD`
- `GO2_SIM_VISUAL_ENV_USD`
- 로컬 RSL-RL / policy checkpoint 경로
- `assets/objects/` 아래 로컬 object asset 경로

대용량 또는 머신 로컬 파일은 GitHub에서 제외합니다.

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

## Asset 및 Checkpoint 주의사항

GitHub에는 `assets/envs/` 아래의 경량 USD 환경 파일만 포함합니다. 대용량 object asset, 생성된 map, 로컬 checkpoint, 로컬 assistant 설정 파일은 의도적으로 제외했습니다.

스크립트가 제외된 asset을 참조하는 경우, 실행 전에 필요한 파일을 예상 경로에 배치하거나 해당 환경 변수를 수정해야 합니다.

## License and Attribution

이 저장소는 [leesj24601/Go2_Intelligence_Framework](https://github.com/leesj24601/Go2_Intelligence_Framework)를 기반으로 수정되었습니다. 원본 저장소와 third-party package, asset의 라이선스를 별도로 확인해야 합니다.

이 로컬 저장소의 커스텀 변경 사항에는 아직 별도의 루트 라이선스를 정의하지 않았습니다. 개인 포트폴리오 범위를 넘어 재배포하기 전에는 upstream license를 검토하고 필요하면 명시적인 루트 `LICENSE` 파일을 추가하는 것이 좋습니다.
