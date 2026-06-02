# Project: Isaac Sim Go2 ORB-SLAM3 Integration

## 0. 사전 환경 검증 (완료)
- [x] Isaac Sim 2023.1 정상 실행 확인 (Omniverse Launcher) -> **Conda Env 'lab' verified**
- [x] Isaac Sim 내 ROS2 Bridge Extension 활성화 확인 (`omni.isaac.ros2_bridge`) -> **Implicitly verified by package presence**
- [x] ROS2 Humble 환경 소싱 및 `ros2 topic list` 동작 확인
- [x] Go2 USD 에셋 확보 (Nucleus 서버 또는 URDF→USD 변환) -> **Found at ~/Desktop/sj/usd/go2.usd**
- [x] `ros2_orb_slam3` 패키지 빌드 상태 확인 (vocabulary 파일 포함)
- [x] 프로젝트 디렉토리 구조 생성 (scripts/, config/, launch/)

## 1. Environment & Robot Setup (진행 중)
- [x] Isaac Sim Standalone Python 스크립트 기본 구조 작성
- [x] Physics Scene 생성 (gravity, time step 설정)
- [x] Ground Plane 추가
- [x] 장애물 환경 구성 (Cylinder/Cube pillars, 벽 등)
- [x] Unitree Go2 USD 로드 및 스폰
- [ ] ArticulationController 설정 (joint position/velocity 제어) -> *Using Root Velocity for SLAM testing*
- [ ] Keyboard Control 구현 (WASD 이동, QE 회전)
- [ ] 로봇 보행 안정성 확인 (넘어지지 않는것)

## 2. ROS2 Bridge & OmniGraph Setup
- [ ] OmniGraph 생성 (Action Graph)
- [ ] ROS2 Clock Publisher 노드 추가 (`/clock`)
- [ ] ROS2 JointState Publisher 노드 추가 (선택)
- [ ] TF Tree Publisher 설정 (`base_link` → `camera_link` 등)
- [ ] 시뮬레이션 시작 시 ROS2 토픽 발행 확인 (`ros2 topic list`)

## 3. Realsense Camera Integration
- [ ] Go2 Head/Face 링크에 USD Camera 부착 (prim path 확인)
- [ ] 카메라 해상도/FOV/FPS 설정 (640x480, 30fps)
- [ ] OmniGraph에 ROS2 Camera Helper 노드 추가
  - [ ] RGB 토픽 발행 (`/camera/color/image_raw`)
  - [ ] Depth 토픽 발행 (`/camera/depth/image_rect_raw`)
  - [ ] CameraInfo 토픽 발행 (`/camera/camera_info`)
- [ ] 카메라 Intrinsics 파라미터 추출 (fx, fy, cx, cy)
- [ ] RViz2에서 RGB/Depth 이미지 표시 확인
- [ ] 카메라가 로봇 머리와 함께 움직이는지 확인

## 4. ORB-SLAM3 Configuration
- [ ] ORB-SLAM3 모드 결정 (RGBD 권장)
- [ ] Isaac Sim 카메라 intrinsics 기반 `.yaml` 설정 파일 작성
- [ ] ORB vocabulary 파일 경로 확인 (`ORBvoc.txt`)
- [ ] 토픽 이름 매핑 정리 (Isaac Sim → ORB-SLAM3)
- [ ] launch 파일 작성 (`go2_slam.launch.py`)
- [ ] ORB-SLAM3 단독 실행 테스트 (토픽 연결 없이 크래시 확인)

## 5. Full Integration & Verification
- [ ] 전체 시스템 순차 실행 스크립트/문서 작성
  1. Isaac Sim (Env + Robot + Camera + ROS Bridge)
  2. RViz2 (이미지 + Map 시각화)
  3. ORB-SLAM3 노드
- [ ] 로봇 키보드 조작으로 환경 탐색
- [ ] ORB-SLAM3 Feature 추출 동작 확인
- [ ] RViz2에서 Point Cloud Map 생성 확인
- [ ] Trajectory 경로 시각화 확인
- [ ] 알려진 이슈 및 해결 방법 문서화
