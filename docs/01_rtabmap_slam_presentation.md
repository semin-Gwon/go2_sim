# Isaac Sim 기반 Go2 로봇 RTAB-Map SLAM 프로젝트 PPT 구성안

프로젝트 마무리를 위한 프리젠테이션(PPT) 스토리보드 및 구성안입니다. 지시하신 3가지 큰 기준(구현 토대, 실행 흐름, 트러블슈팅)에 맞추어 내용을 다듬고 쫙 정리했습니다.

---

## 1. 구현 토대 (System Foundation)

### ① Simulation Environment (World Building)
**"SLAM 성능 검증의 한계치를 시험하기 위한 최적의 커스텀 환경 구축"**
단순히 빈 평면(Ground Plane)만 로드하는 것을 넘어, SLAM의 특징점 추출 능력을 검증하기 위해 복잡성이 부여된 특수 환경을 제작했습니다.

*   **환경 생성 방식**: Isaac Sim의 MCP(소켓 통신) 기능과 파이썬 스크립트(`deploy_scene_mcp.py`)를 활용하여 다수의 기둥(장애물)이 랜덤하지만 일정한 간격으로 배치되는 실내 환경을 프로시저럴(Procedural) 단계로 자동 생성했습니다.
*   **선정 이유**: 시각 기반 SLAM(RTAB-Map)이 특징점을 잘 잡기 위해서는 배경 텍스처와 물체의 모서리(Edge)가 풍부하게 노출되어야 합니다. 다수의 3D 기둥 장애물을 배치해 비전 데이터의 풍부함을 확보하고, 회전 시 Odometry 드리프트(Drift)나 Loop Closure 방식을 뚜렷하게 테스트하기 위함이었습니다.
*   **자동 로드 적용**: 이렇게 생성한 맵은 하나의 USD(`slam_env.usd`) 패키지로 결합되어, Isaac Lab의 `TerrainImporterCfg` 파이프라인을 통해 물리 엔진 설정(마찰계수 1.0)과 함께 한 줄의 코드로 자동 로드되도록 자동화했습니다.

### ② Robot Configuration (Go2 in Isaac Sim)
**"강화학습 기반의 4족 보행 로봇 제어 구현"**
가상 공간에 스폰된 로봇이 스케이팅(Skating)하지 않고, 실제 중력과 역학을 견디며 땅을 딛고 걷게 만들었습니다.

*   **Object-Oriented Configuration (객체지향 설계 적용)**: 로봇의 관절, 센서, 험지 극복용 물리 파라미터를 처음부터 수동으로 세팅하지 않았습니다. 대신 NVIDIA Isaac Lab이 공식 제공하는 험지 물리 템플릿인 **`UnitreeGo2RoughEnvCfg` 클래스를 통째로 상속(Inheritance)** 받았습니다.
*   **Custom Overriding (커스텀 덮어쓰기)**: 상속받은 부모 로봇 환경을 그대로 유지한 채, 저희는 오직 **(1) 커스텀 지형 장애물 맵 삽입**과 **(2) SLAM용 3D 카메라인 RealSense D435 센서 마운트** 부분만 오버라이딩하여 코드의 안전성과 모듈화를 극대화했습니다.
*   **Motion Control (RL Policy)**: 로봇의 관절을 단순히 ROS 토픽으로 꺾는 방식이 아닌, 사전 학습된 RSL-RL(강화학습) 보행 정책을 로드했습니다. 사용자가 키보드(WASD)로 Twist 커맨드를 주면, RL 에이전트가 이를 해석해 실시간 관절 타겟으로 변환하여 매우 사실적이고 동적인 4족 보행을 구현합니다.

### ③ Sensor Perception (Input Data)
**"Omniverse 기반 고정밀 멀티 모달 센서 융합"**
성공적인 매핑을 위해 Go2의 머리가 되는 부분에 가상의 센서 스택(Sensor Stack)을 정밀하게 장착했습니다.

*   **센서 배치**: Go2의 베이스 링크(Base_link) 중심에서 전면부(직진 방향 30cm, 지면 높이 5cm)를 정확히 계산해 Intel RealSense D435 모델의 시야각(FOV)에 맞춘 가상 카메라를 부착. (PPT 제작 시 `CameraCfg` 부착된 로봇 정면의 스크린샷 활용 권장)
*   **데이터 사양**:
    *   **RGB-D (Vision)**: 프레임 드랍을 막기 위해 렌더링 해상도는 320x240 또는 640x480으로 최적화하였으며, Isaac Sim 물리 틱(30fps) 중 프레임 스킵을 적용해 안정적인 10Hz 속도로 이미지와 Depth를 출력.
    *   **IMU & Odometry**: 50Hz의 가속도/자이로 센서를 부착하고, 6DoF의 정밀 Ground Truth 오도메트리를 산출해 시점 변환(TF) 오류를 차단.

---

## 2. 실행 흐름 (Data Flow Block Diagram)

"시뮬레이터에서 만들어진 데이터가, 어떻게 3D 맵으로 재탄생하는가?"
이 섹션은 도식화(Block Diagram) 형태의 슬라이드로 표현하면 매우 효과적입니다.

**[ Control Loop (제어 흐름) ]**
➡️ `User (WASD)` ➡️ `Isaac Lab (RL-Policy)` ➡️ `Joint Actuation` ➡️ `Go2 Movement`

**[ Data Pipeline (위의 이동으로 인한 센서 데이터 흐름) ]**
*   **Input Stage (Isaac Sim / OmniGraph)**
    *   내장된 **ROS2 Bridge Extension**을 통해 시뮬레이션 데이터를 실시간 변환.
    *   RGB/Depth 렌더 이미지 ➡️ `/camera/color/image_raw`, `/camera/depth/image_rect_raw`
    *   위치/자세 데이터 ➡️ `/odom`, `/imu/data`, `/tf`
*   **Processing Stage (ROS2 RTAB-Map Node)**
    *   **Odometry Estimation**: 시각 데이터와 IMU를 융합하여 로봇의 누적 궤적 계산
    *   **Loop Closure Detection**: 이전 방문했던 장소를 인식해 누적된 맵의 오차율(Drift) 보정
    *   **Grid Mapping**: `Grid/FromDepth: true` 파라미터로 3D 포인트 클라우드를 2D 평면 Occupancy Grid로 누름(Projection).
*   **Output Stage (RViz2 Visualization)**
    *   `MapData` Topic ➡️ 실시간 점군(Point Cloud) 3D 공간 시각화
    *   `/grid_map` Topic ➡️ 2D 장애물 맵(네비게이션용) 시각화
    *   `/tf` ➡️ 로봇의 실시간 지나온 Trajectory(궤적) 드로잉

---

## 3. 트러블슈팅 (Troubleshooting)

이 프로젝트의 진면목(기술적 극복 과정)을 보여주는 가장 중요한 하이라이트 파트입니다.

### 💣 1. 동기화 및 타임스탬프 불일치 (Time Domain Mismatch)
*   **현상**: Isaac Sim의 물리 계산 시간과 외부 ROS2 시스템의 컴퓨터 벽시계(Wall-clock) 시간이 달라 TF(Transform) 트리가 찢어지고 맵이 생성되지 않음.
*   **삽질 및 극복**: `ROS2PublishClock` OmniGraph 노드를 추가해 `/clock` 토픽을 강제로 발행하고, RTAB-Map 런치 파이프라인 전체에 `use_sim_time:=true`를 부여했습니다. 추가로, 센서 퍼블리시 단계를 `ONDEMAND` 방식 대신 물리엔진 스텝에 완벽히 연동되는 `SIMULATION` 틱으로 전환하여 "시간 동기화율 100%"를 달성했습니다.

### 💣 2. 3D 맵의 부채꼴 왜곡 및 고스트 레이어 (Voxel Map Tilt & Ghosting)
*   **현상**: 로봇이 직진을 할 때 맵의 바닥이 평편하지 않고 여러 각도의 레이어로 계속 겹쳐서 생성(Ghost Layer)되는 치명적 표류(Drift) 발생. 
*   **삽질 및 극복**: 4족 보행 특성상 어쩔 수 없이 발생하는 Pitch/Roll 진동 흔들림을 RTAB-Map이 2D 바퀴형 로봇용 알고리즘(3DoF)으로 억지로 최적화하려다 발생한 문제임을 분석해 냄. RTAB-Map의 핵심 파라미터인 `Reg/Force3DoF`를 평면용 제약인 `true`에서 `false`로 과감히 변경. 6DoF의 흔들림 궤적 자체를 전부 수용하도록 만들어 완벽하고 평평한 맵을 그려냈습니다.

### 💣 3. 프레임 드랍과 리소스 병목 (Resource Bottleneck)
*   **현상**: GPU 연산량이 폭발하여 베이스 뷰포트 화면과 카메라 뷰포트 시뮬레이션이 모두 멈칫거리거나(Lag) 통신 프레임 드랍이 발생.
*   **삽질 및 극복**: 보이지 않는(Hidden) 오프스크린 뷰포트를 따로 생성하고 `update_period=0`, 카메라 렌더 해상도를 320x240으로 대폭 축소. 더 나아가 영상의 전송 주기(FrameSkipCount)를 2로 설정하여 30FPS의 시뮬레이션을 10Hz의 부담 없는 전송망으로 최적화하였습니다. 또한 파이썬 3.11 환경과 `rclpy` 패키지의 충돌 한계를 인정하고, 우회 타개책으로 "Isaac Sim 내장 OmniGraph Bridge" 노드 프로그래밍 구조를 채택하여 완벽히 문제를 돌파했습니다.
