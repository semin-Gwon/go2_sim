# my_slam_env.py
import os
from isaaclab.utils import configclass
from isaaclab.assets import AssetBaseCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.sensors import CameraCfg, ImuCfg
import isaaclab.sim as sim_utils
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_tasks.manager_based.locomotion.velocity.config.go2.rough_env_cfg import (
    UnitreeGo2RoughEnvCfg,
)


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


def _env_str(name: str, default: str) -> str:
    value = os.environ.get(name)
    return value if value else default


@configclass
class MySlamEnvCfg(UnitreeGo2RoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        visual_env_usd = _env_str("GO2_SIM_VISUAL_ENV_USD", "/home/jnu/go2_sim/assets/envs/room_env.usd")
        visual_env_pos = (
            _env_float("GO2_SIM_VISUAL_ENV_X", 0.0),
            _env_float("GO2_SIM_VISUAL_ENV_Y", 0.0),
            _env_float("GO2_SIM_VISUAL_ENV_Z", 0.0),
        )
        robot_spawn = (
            _env_float("GO2_SIM_ROBOT_SPAWN_X", 0.0),
            _env_float("GO2_SIM_ROBOT_SPAWN_Y", 0.0),
            _env_float("GO2_SIM_ROBOT_SPAWN_YAW", 0.0),
        )

        # 1. 로봇이 항상 설 수 있도록 확실한 평면 지면을 먼저 생성
        self.scene.terrain = TerrainImporterCfg(
            prim_path="/World/ground",
            terrain_type="plane",
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="multiply",
                restitution_combine_mode="multiply",
                static_friction=1.5,
                dynamic_friction=1.5,
            ),
        )

        # 1-1. 커스텀 환경 USD는 시각/장애물 장면으로 별도 로드
        self.scene.visual_env = AssetBaseCfg(
            prim_path="/World/visual_env",
            spawn=sim_utils.UsdFileCfg(
                usd_path=visual_env_usd,
            ),
            init_state=AssetBaseCfg.InitialStateCfg(pos=visual_env_pos),
            collision_group=-1,
        )

        # 2. 커스텀 USD는 프림 구조가 제각각이라 rough-env의 기본 height scanner가
        #    기대하는 /World/ground Mesh가 없을 수 있다. 이 프로젝트는 이미
        #    policy obs에서 height_scan을 제거했으므로 센서를 비활성화한다.
        self.scene.height_scanner = None

        # 3. 제어 설정 유지
        if hasattr(self.commands, "base_velocity"):
            self.commands.base_velocity.resampling_time_range = (1.0e9, 1.0e9)
            self.commands.base_velocity.debug_vis = False
            # [중요] Heading command를 꺼야 사용자의 Q/E 회전 명령이 직접 전달됩니다.
            self.commands.base_velocity.heading_command = False

        self.episode_length_s = 1.0e9
        if hasattr(self.curriculum, "terrain_levels"):
            self.curriculum.terrain_levels = None

        # Navigation/localization 재현성을 위해 reset 시 로봇 시작 위치/자세를 고정한다.
        if hasattr(self.events, "reset_base"):
            self.events.reset_base.params = {
                "pose_range": {
                    "x": (robot_spawn[0], robot_spawn[0]),
                    "y": (robot_spawn[1], robot_spawn[1]),
                    "yaw": (robot_spawn[2], robot_spawn[2]),
                },
                "velocity_range": {
                    "x": (0.0, 0.0),
                    "y": (0.0, 0.0),
                    "z": (0.0, 0.0),
                    "roll": (0.0, 0.0),
                    "pitch": (0.0, 0.0),
                    "yaw": (0.0, 0.0),
                },
            }

        # Unitree RL Lab 정책 obs space 맞추기 (45-dim)
        # 제거: base_lin_vel(3), height_scan(~187)
        # 유지: base_ang_vel(3×0.2), projected_gravity(3), velocity_commands(3),
        #        joint_pos_rel(12), joint_vel_rel(12×0.05), last_action(12)
        self.observations.policy.base_lin_vel = None
        self.observations.policy.height_scan = None
        self.observations.policy.base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel, scale=0.2, noise=Unoise(n_min=-0.2, n_max=0.2)
        )
        self.observations.policy.joint_vel = ObsTerm(
            func=mdp.joint_vel_rel, scale=0.05, noise=Unoise(n_min=-1.5, n_max=1.5)
        )

        # IMU 센서 (50Hz, body frame)
        self.scene.imu_sensor = ImuCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base",
            update_period=1.0 / 50.0,
            gravity_bias=(0.0, 0.0, 9.81),
        )

        # Intel RealSense D435 근사 카메라
        self.scene.front_cam = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base/front_cam",
            update_period=0,  # 센서 데이터 수집 비활성화 (ROS2는 숨겨진 뷰포트 사용)
            height=240,
            width=320,
            data_types=[],  # prim만 생성, 센서 렌더링 안 함 (이중 렌더링 방지)
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=15.0,
                focus_distance=400.0,
                horizontal_aperture=20.955,
                clipping_range=(0.1, 50.0),
            ),
            offset=CameraCfg.OffsetCfg(
                pos=(0.30, 0.0, 0.05),
                rot=(0.5, -0.5, 0.5, -0.5),
                convention="ros",
            ),
        )
