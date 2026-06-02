# Phase 6: 실로봇 배포

> 사전 조건: unitree_rl_lab 훈련 완료 + 시뮬 검증 완료 (`docs/03_nav2_plan.md` Phase 0~5)

---

## 전체 구조

시뮬의 `go2_sim.py` 역할을 실로봇에서는 `unitree_rl_lab deploy`가 대신한다.
두 프로세스가 반드시 동시에 실행되어야 한다.

```
[시뮬]                              [실로봇]
go2_sim.py (Isaac Sim)          =   unitree_rl_lab deploy (C++)
  └─ RL policy → 관절 제어            └─ RL policy → 관절 제어 (DDS)

Nav2 + RTAB-Map (동일)          =   Nav2 + RTAB-Map (동일)
  └─ /cmd_vel 발행                    └─ /cmd_vel 발행
```

### 실행 구조
```bash
# 로봇 PC (또는 온보드)
./go2_ctrl                              # unitree_rl_lab deploy → 관절 제어

# 노트북
ros2 launch go2_navigation_real.launch.py   # Nav2 + RTAB-Map → /cmd_vel 발행
```

### 통신 레이어
```
[deploy 코드]   DDS ↔ 로봇 하드웨어 (관절/IMU/조이스틱)
                + rclcpp 추가 예정 → /cmd_vel 수신

[ROS2 브릿지]   unitree_ros2 패키지
                DDS LowState → /odom, /tf 발행 → Nav2 수신
```

> ℹ️ unitree_ros2 브릿지는 Nav2가 로봇 위치를 알기 위해 필요.
> 설치/실행 여부 및 토픽명은 로봇 연결 후 확인 필요.

---

## RL 정책 obs 구조 및 cmd_vel 연동 원리

RL 정책 obs 벡터에는 **velocity_commands (vx, vy, omega)** 가 포함된다.
현재는 조이스틱이 이 값을 채워주고, Nav2는 이 자리를 `/cmd_vel`로 대체한다.

```cpp
// deploy/include/isaaclab/envs/mdp/observations/observations.h
REGISTER_OBSERVATION(velocity_commands)
{
    auto & joystick = env->robot->data.joystick;
    obs[0] = clamp(joystick->ly(), ...);    // vx  ← Nav2 cmd_vel.linear.x 로 대체
    obs[1] = clamp(-joystick->lx(), ...);   // vy  ← Nav2 cmd_vel.linear.y 로 대체
    obs[2] = clamp(-joystick->rx(), ...);   // omega ← Nav2 cmd_vel.angular.z 로 대체
    return obs;
}
```

**수정 방향**: cmd_vel 수신 중이면 cmd_vel 우선, 없으면 조이스틱 폴백
```cpp
if (cmd_vel_available && !timeout) {
    obs[0] = clamp(cmd_vel_vx,    -0.5, 1.0);
    obs[1] = clamp(cmd_vel_vy,    -0.4, 0.4);
    obs[2] = clamp(cmd_vel_omega, -1.0, 1.0);
} else {
    obs[0] = clamp(joystick->ly(), ...);   // 폴백
    obs[1] = clamp(-joystick->lx(), ...);
    obs[2] = clamp(-joystick->rx(), ...);
}
```

---

## 시뮬 → 실로봇 변경 사항

| 항목 | 시뮬 | 실로봇 |
|------|------|--------|
| 관절 제어 | go2_sim.py (Isaac Sim) | unitree_rl_lab deploy (C++, DDS) |
| 카메라 | OmniGraph 가상 카메라 | RealSense D435i 드라이버 |
| /odom, /tf | OmniGraph 계산 → ROS2 | unitree_ros2 브릿지 → ROS2 |
| /cmd_vel 수신 | go2_sim.py CmdVelNode (Python) | deploy observations.h 수정 (C++) |
| 클록 | /clock (sim time) | 시스템 시간 |
| use_sim_time | true | false |
| RViz | go2_sim.rviz | go2_sim.rviz (동일) |

---

## Phase A: cmd_vel 수신 구현 (unitree_rl_lab deploy)

> 수정 파일: `deploy/include/isaaclab/envs/mdp/observations/observations.h`
> 현재 deploy 코드에 ROS2 없음 → rclcpp 추가 필요

- [ ] `CMakeLists.txt`에 `rclcpp`, `geometry_msgs` 의존성 추가
- [ ] 글로벌 cmd_vel 수신 스레드 추가 (`rclcpp::init` + `spin_some` 루프, DDS 루프와 병행)
- [ ] `velocity_commands` observation 수정
  - cmd_vel 수신 시: cmd_vel 값으로 obs 채움 (clamp 적용)
  - 0.5초 타임아웃 시: 조이스틱 폴백
- [ ] 빌드 확인

---

## Phase B: 로봇 연결 후 확인 항목

> ⚠️ 아래 항목은 로봇 연결 전에는 알 수 없음. 연결 즉시 확인.

```bash
ros2 topic list         # /odom 토픽명, /tf 발행 여부 확인
ros2 run tf2_ros tf2_monitor    # odom → base_link TF 발행 주체 확인
```

- [ ] unitree_ros2 브릿지 설치/실행 여부 확인
- [ ] `/odom` 토픽명 확인 → `go2_nav2_params_real.yaml` `odom_topic` 반영
- [ ] `odom → base_link` TF 발행 주체 확인
- [ ] RealSense 드라이버 토픽명 확인 → `go2_rtabmap.launch.py` remapping과 일치 여부

---

## Phase C: Launch 파일 및 파라미터 (실로봇 전용)

### `launch/go2_navigation_real.launch.py` (신규)
- [ ] `use_sim_time: false` 고정
- [ ] RealSense 드라이버 노드 포함 (`realsense2_camera`)
- [ ] `/clock` 퍼블리셔 제거
- [ ] `go2_nav2_params_real.yaml` 연결

### `config/go2_nav2_params_real.yaml` (신규)
- [ ] `use_sim_time: false`
- [ ] Phase B 확인 후 `odom_topic` 반영
- [ ] 초기 속도 제한 보수적 설정 (안정 확인 후 점진적으로 올림)

| 파라미터 | 시뮬 | 실로봇 초기값 |
|---------|------|------------|
| `use_sim_time` | true | **false** |
| `vx_max` | 1.0 | **0.5** |
| `vy_max` | 0.4 | **0.2** |
| `wz_max` | 1.0 | **0.5** |
| `max_accel` | [2.5, 2.5, 3.2] | **[1.5, 1.5, 2.0]** |

---

## Phase D: 통합 테스트 절차

```
1. deploy 코드 빌드 및 단독 실행
   └─ 로봇 기립/정지 확인 (Nav2 없이)

2. cmd_vel 수동 테스트
   └─ ros2 topic pub /cmd_vel ... → 로봇 이동 확인
   └─ 0.5초 후 타임아웃 → 자동 정지 확인

3. RealSense + RTAB-Map 맵 생성
   └─ ros2 launch go2_rtabmap.launch.py use_sim_time:=false
   └─ 수동 조종하며 실내 맵 생성 → maps/rtabmap_real.db 저장

4. Nav2 자율주행 테스트
   └─ ros2 launch go2_navigation_real.launch.py
   └─ RViz에서 Goal Pose 지정 → 이동 확인

5. 속도 점진적 상향
   └─ 0.5 → 0.7 → 1.0 m/s (안정 확인 후)
```

---

## 속도 파라미터 참고

현재 `config/go2_nav2_params.yaml` 값은 **unitree_rl_lab `limit_ranges` 기준**:

```yaml
vx_max: 1.0   # limit_ranges x=±1.0
vy_max: 0.4   # limit_ranges y=±0.4
wz_max: 1.0   # limit_ranges ang_z=±1.0
```

> ⚠️ 실로봇 초기 테스트는 Phase C 표의 보수적 값으로 시작. 안정 확인 후 단계적으로 올린다.
