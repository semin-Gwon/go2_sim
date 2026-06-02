# RL 정책 전환: IsaacLab 공식 → unitree_rl_lab

**결정일**: 2026-02-24 | **상태**: ✅ 통합 완료

---

## 한눈에 보기

| 항목 | 기존 (IsaacLab 공식) | 변경 (unitree_rl_lab) |
|---|---|---|
| 소스 | IsaacLab 서버 자동 다운로드 | Unitree 공식 로컬 학습 |
| 태스크 | `Isaac-Velocity-Rough-Unitree-Go2-v0` | `Unitree-Go2-Velocity` |
| 시뮬레이터 | Isaac Lab / PhysX | Isaac Lab 2.3.0 / PhysX (동일) |
| actor obs | ~235-dim (height_scan 포함) | **45-dim** |
| 실로봇 배포 | ❌ 없음 | ✅ `deploy/robots/go2/` 포함 |

---

## 전환 이유

1. **실로봇 배포**: 최종 목표는 실제 Go2에 Nav2 자율주행을 올리는 것.
   IsaacLab 공식 정책은 실로봇 배포 파이프라인이 없고, unitree_rl_lab은 `deploy/` 코드가 포함됨.

2. **Sim-to-Sim 갭 없음**: 둘 다 Isaac Lab / PhysX 기반이므로 별도 검증 불필요. 체크포인트만 교체하면 됨.

3. **동일 기술 스택**: RSL-RL 기반이어서 기존 `go2_sim.py` 파이프라인 변경 최소화.

---

## unitree_rl_lab 설정

### 경로
```
/home/cvr/Desktop/sj/unitree_rl_lab/
```

### USD 모델 경로 (설치 시 1회 설정)
- 파일: `source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py`
- `UNITREE_MODEL_DIR = "/home/cvr/Desktop/sj/unitree_model"`

### 학습 명령
```bash
cd /home/cvr/Desktop/sj/unitree_rl_lab
python scripts/rsl_rl/train.py --headless --task Unitree-Go2-Velocity
```

### 학습 재개
```bash
python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity \
  --headless \
  --resume \
  --load_run <세션명>    # 예: 2026-02-24_14-26-01
  --checkpoint <모델명>  # 예: model_41600.pt
```

### 검증 (play)
```bash
./unitree_rl_lab.sh -p --task Unitree-Go2-Velocity
```

### 학습 로그 경로
```
logs/rsl_rl/unitree_go2_velocity/
├── 2026-02-23_17-47-48/
├── 2026-02-24_12-51-50/
├── 2026-02-24_12-56-05/
└── 2026-02-24_14-26-01/   ← 현재 활성 세션 (학습 진행 중)
```

---

## go2_sim.py 통합 구현 ✅

변경된 파일: `scripts/go2_sim.py`, `scripts/my_slam_env.py`

### 1. 체크포인트 자동 탐색 (`go2_sim.py`)

학습 진행 중에도 실행 시마다 최신 체크포인트를 자동 선택:

```python
_log_dir = "/home/cvr/Desktop/sj/unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity"
_sessions = sorted(glob.glob(os.path.join(_log_dir, "*")))   # 날짜순
_pts = sorted(..., key=lambda p: int(re.search(r"model_(\d+)\.pt", p).group(1)))
resume_path = _pts[-1]  # 번호 최신
```

### 2. obs space 수정 (`my_slam_env.py`)

`UnitreeGo2RoughEnvCfg`의 기본 obs에서 unitree_rl_lab 정책(45-dim)에 맞게 오버라이드:

| obs 항목 | 기본 | unitree_rl_lab | 조치 |
|---|---|---|---|
| `base_lin_vel` (3) | ✅ | ❌ | 제거 |
| `height_scan` (~187) | ✅ | ❌ | 제거 |
| `base_ang_vel` (3) | scale 없음 | scale=0.2 | 재정의 |
| `joint_vel` (12) | scale 없음 | scale=0.05 | 재정의 |
| `projected_gravity`, `velocity_commands`, `joint_pos_rel`, `last_action` | ✅ | ✅ | 유지 |
| **합계** | **~235-dim** | **45-dim** | |

### 3. 가중치 로드 (`go2_sim.py`)

unitree_rl_lab은 actor(45-dim) / critic(60-dim) obs를 분리 학습함.
현재 env에 critic obs group이 없으므로 → critic 가중치 제외 후 로드:

```python
_ckpt = torch.load(resume_path, weights_only=False)
_actor_state = {k: v for k, v in _ckpt["model_state_dict"].items()
                if not k.startswith("critic")}
runner.alg.policy.load_state_dict(_actor_state, strict=False)
```

> **왜 `strict=False`만으론 안 되나?**
> PyTorch는 키가 존재하는데 크기가 다르면 strict 여부와 관계없이 에러를 냄.
> critic 키 자체를 dict에서 제거해야 "없는 키 → 무시(strict=False)"로 처리됨.

> **critic obs 60-dim 구성** (추론 시 미사용):
> 기본 45-dim + `base_lin_vel`(3) + `joint_effort`(12) = 60-dim

---

## 관련 문서

- `03_nav2_plan.md` — Nav2 자율주행 구현 계획
