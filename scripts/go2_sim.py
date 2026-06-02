#!/usr/bin/env python3
import os
import sys

# Isaac Sim 번들 Humble rclpy(Python 3.11용)를 가장 먼저 등록.
# source /opt/ros/humble/setup.bash 로 잡힌 시스템 Python 3.10 경로보다
# 앞에 위치시켜야 모든 ROS2 패키지가 번들에서 로딩됨.
_ISAAC_ROS2_PATH = (
    "/home/jnu/anaconda3/envs/isaaclab/lib/python3.11/site-packages"
    "/isaacsim/exts/isaacsim.ros2.bridge/humble/rclpy"
)
_ISAAC_ROS_LIB_PATH = (
    "/home/jnu/anaconda3/envs/isaaclab/lib/python3.11/site-packages"
    "/isaacsim/exts/isaacsim.ros2.bridge/humble/lib"
)
_CONDA_LIB_PATH = os.path.join(sys.prefix, "lib")

sys.path = [p for p in sys.path if not p.startswith("/opt/ros/")]
if _ISAAC_ROS2_PATH not in sys.path:
    sys.path.insert(0, _ISAAC_ROS2_PATH)

# Isaac Sim ROS2 Bridge는 프로세스 시작 시점의 라이브러리 경로를 사용한다.
_ros_loader_paths = [p for p in (_ISAAC_ROS_LIB_PATH, _CONDA_LIB_PATH) if os.path.isdir(p)]
if _ros_loader_paths:
    ld_library_entries = [entry for entry in os.environ.get("LD_LIBRARY_PATH", "").split(":") if entry]
    missing_loader_paths = [entry for entry in _ros_loader_paths if entry not in ld_library_entries]
    if missing_loader_paths:
        os.environ["LD_LIBRARY_PATH"] = ":".join([*missing_loader_paths, *ld_library_entries])
        if os.environ.get("GO2_SIM_ROS_ENV_READY") != "1":
            os.environ["GO2_SIM_ROS_ENV_READY"] = "1"
            os.execvpe(sys.executable, [sys.executable, *sys.argv], os.environ)

import argparse
import json
import time
import logging
import math
import yaml
import threading
from pathlib import Path
import numpy as np
import torch
import gymnasium as gym

# Isaac Sim 경고 로그 필터링
logging.getLogger("isaacsim").setLevel(logging.ERROR)
logging.getLogger("omni").setLevel(logging.ERROR)

from isaaclab.app import AppLauncher

# 0. Pre-parse --rt argument (before AppLauncher/Hydra)
rt_mode = "true"
argv_copy = sys.argv.copy()
for i, arg in enumerate(argv_copy):
    if arg == "--rt" and i + 1 < len(argv_copy):
        rt_mode = argv_copy[i + 1].lower()
        # Remove --rt and its value from sys.argv so Hydra doesn't see it
        sys.argv = argv_copy[:i] + argv_copy[i + 2 :]
        break
    elif arg.startswith("--rt="):
        rt_mode = arg.split("=", 1)[1].lower()
        sys.argv = argv_copy[:i] + argv_copy[i + 1 :]
        break

# 1. Setup Parser
parser = argparse.ArgumentParser(description="Go2 Simulation matching run_slam style")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
parser.add_argument(
    "--task",
    type=str,
    default="Isaac-Velocity-Rough-Unitree-Go2-Play-v0",
    help="Task name.",
)
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    default=True,
    help="Use checkpoint.",
)

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import cli_args

cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
args_cli.enable_cameras = True
args_cli.rt = rt_mode

# Launch simulation app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# 2. Imports after app launch
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from isaaclab.devices import Se2Keyboard, Se2KeyboardCfg
from isaaclab.utils.pretrained_checkpoint import get_published_pretrained_checkpoint
from rsl_rl.runners import OnPolicyRunner
from my_slam_env import MySlamEnvCfg
from isaaclab_tasks.utils.hydra import hydra_task_config
import isaaclab_tasks  # noqa

import omni.graph.core as og
from isaacsim.core.utils import extensions
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

# ROS2 bridge 확장 활성화
extensions.enable_extension("isaacsim.ros2.bridge")

simulation_app.update()

VISIBLE_OBJECTS_CONFIG_PATH = os.environ.get(
    "GO2_SIM_VISIBLE_OBJECTS_CONFIG",
    "/home/jnu/llm_yolo/configs/sim/sim_visible_objects.json",
)
NAMED_PLACES_CONFIG_PATH = os.environ.get(
    "GO2_SIM_NAMED_PLACES_CONFIG",
    "/home/jnu/llm_yolo/configs/sim/sim_named_places.yaml",
)
VISUAL_ENV_COLLISIONS = os.environ.get("GO2_SIM_VISUAL_ENV_COLLISIONS", "true").lower() in (
    "1",
    "true",
    "yes",
    "on",
)
VISUAL_ENV_DISABLE_FLOOR_COLLISIONS = os.environ.get(
    "GO2_SIM_DISABLE_VISUAL_FLOOR_COLLISIONS", "true"
).lower() in ("1", "true", "yes", "on")

DYN_PERSON_ENABLED = os.environ.get("GO2_SIM_DYNAMIC_PERSON_ENABLED", "true").lower() in (
    "1",
    "true",
    "yes",
    "on",
)
DYN_PERSON_A = (-1.8, -2.2, 0.0)
DYN_PERSON_B = (-1.8, 1.8, 0.0)
DYN_PERSON_SPEED_MPS = 0.25
DYN_PERSON_RADIUS_M = 0.22
DYN_PERSON_HEIGHT_M = 1.20
DYN_PERSON_USD_PATH = os.environ.get(
    "GO2_SIM_DYNAMIC_PERSON_USD",
    "/home/jnu/go2_sim/assets/objects/male_adult/male_adult_construction_03.usd",
).strip()
DYN_PERSON_VISUAL_SCALE = float(os.environ.get("GO2_SIM_DYNAMIC_PERSON_SCALE", "0.7"))
DYN_PERSON_VISUAL_Z_OFFSET = float(os.environ.get("GO2_SIM_DYNAMIC_PERSON_Z_OFFSET", "0.0"))
DYN_PERSON_VISUAL_YAW_OFFSET_DEG = float(os.environ.get("GO2_SIM_DYNAMIC_PERSON_YAW_OFFSET_DEG", "0.0"))


def load_visible_object_specs(config_path: str):
    path = Path(config_path)
    if not path.exists():
        print(f"[WARN] visible objects config not found: {path}")
        return []
    try:
        data = json.loads(path.read_text())
    except Exception as exc:
        print(f"[WARN] visible objects config parse failed: {exc}")
        return []
    specs = []
    for item in data.get("visible_objects", []):
        try:
            specs.append({
                "class_name": str(item["class_name"]),
                "prim_path": str(item["prim_path"]),
                "radius_m": float(item.get("radius_m", 1.0)),
            })
        except Exception as exc:
            print(f"[WARN] skipping visible object spec {item}: {exc}")
    print(f"[INFO] visible object specs loaded: {len(specs)} from {path}")
    return specs


def find_prim_by_basename(stage, prim_path: str):
    base_name = str(prim_path).rstrip("/").split("/")[-1]
    if not base_name:
        return None
    for prim in stage.Traverse():
        if prim.GetName() == base_name:
            return prim
    return None


def compute_visible_objects(robot_x: float, robot_y: float, specs, stage):
    visible = []
    debug_rows = []
    for spec in specs:
        prim = stage.GetPrimAtPath(spec["prim_path"])
        if not prim.IsValid():
            fallback_prim = find_prim_by_basename(stage, spec["prim_path"])
            if fallback_prim is None or not fallback_prim.IsValid():
                debug_rows.append(f"{spec['class_name']}: invalid_prim")
                continue
            prim = fallback_prim
        try:
            imageable = UsdGeom.Imageable(prim)
            transform = imageable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            translation = transform.ExtractTranslation()
            obj_x = float(translation[0])
            obj_y = float(translation[1])
        except Exception as exc:
            debug_rows.append(f"{spec['class_name']}: pose_error={exc}")
            continue
        dx = robot_x - obj_x
        dy = robot_y - obj_y
        dist = (dx * dx + dy * dy) ** 0.5
        debug_rows.append(
            f"{spec['class_name']}: obj=({obj_x:.3f},{obj_y:.3f}) dist={dist:.3f} r={spec['radius_m']:.3f}"
        )
        if dist <= spec["radius_m"]:
            visible.append(spec["class_name"])
    return sorted(set(visible)), debug_rows


def load_named_place_specs(config_path: str):
    path = Path(config_path)
    if not path.exists():
        print(f"[WARN] named places config not found: {path}")
        return {}
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except Exception as exc:
        print(f"[WARN] named places config parse failed: {exc}")
        return {}
    specs = data.get("named_places", {})
    print(f"[INFO] named place specs loaded: {len(specs)} from {path}")
    return specs


def compute_named_place_poses(specs, stage):
    poses = {}
    for name, spec in specs.items():
        target = dict(spec or {})
        prim_path = target.get("prim_path")
        if prim_path:
            prim = stage.GetPrimAtPath(str(prim_path))
            if not prim.IsValid():
                fallback_prim = find_prim_by_basename(stage, str(prim_path))
                if fallback_prim is not None and fallback_prim.IsValid():
                    prim = fallback_prim
                elif str(name) == "center":
                    prim = stage.GetPrimAtPath("/World/visual_env")
                else:
                    continue
            try:
                imageable = UsdGeom.Imageable(prim)
                transform = imageable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                translation = transform.ExtractTranslation()
                target["x_m"] = float(translation[0])
                target["y_m"] = float(translation[1])
            except Exception:
                continue
        if "x_m" in target and "y_m" in target:
            poses[str(name)] = {
                "x_m": float(target["x_m"]),
                "y_m": float(target["y_m"]),
                "yaw_rad": float(target.get("yaw_rad", 0.0)),
                "radius_m": float(target.get("radius_m", 0.5)),
            }
    return poses


def describe_stage_prim(stage, prim_path: str):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return {"path": prim_path, "valid": False}
    children = prim.GetChildren()
    summary = {
        "path": prim_path,
        "valid": True,
        "type": prim.GetTypeName(),
        "children": len(children),
        "child_names": [child.GetName() for child in children[:8]],
    }
    try:
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        world_bbox = bbox_cache.ComputeWorldBound(prim)
        bbox_range = world_bbox.ComputeAlignedBox()
        if not bbox_range.IsEmpty():
            minimum = bbox_range.GetMin()
            maximum = bbox_range.GetMax()
            center = (minimum + maximum) * 0.5
            size = maximum - minimum
            summary["bbox_center"] = [float(center[0]), float(center[1]), float(center[2])]
            summary["bbox_size"] = [float(size[0]), float(size[1]), float(size[2])]
    except Exception as exc:
        summary["bbox_error"] = str(exc)
    return summary


def find_camera_anchor(stage, candidate_paths: list[str]):
    for prim_path in candidate_paths:
        summary = describe_stage_prim(stage, prim_path)
        if summary.get("valid") and "bbox_center" in summary and "bbox_size" in summary:
            return prim_path, summary
    return None, None


def find_descendant_with_bbox(stage, root_path: str, max_visited: int = 2000):
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None, None

    queue = [root]
    visited = 0
    while queue and visited < max_visited:
        prim = queue.pop(0)
        visited += 1
        prim_path = prim.GetPath().pathString
        summary = describe_stage_prim(stage, prim_path)
        if summary.get("valid") and "bbox_center" in summary and "bbox_size" in summary:
            size = summary["bbox_size"]
            if max(size) > 0.01:
                return prim_path, summary
        queue.extend(list(prim.GetChildren()))
    return None, None


def set_robot_spawn_to_xy(env, x: float, y: float, yaw: float = 0.0):
    if not hasattr(env.unwrapped, "cfg"):
        return False
    events = getattr(env.unwrapped.cfg, "events", None)
    reset_base = getattr(events, "reset_base", None) if events is not None else None
    if reset_base is None or not hasattr(reset_base, "params"):
        return False

    reset_base.params = {
        "pose_range": {"x": (x, x), "y": (y, y), "yaw": (yaw, yaw)},
        "velocity_range": {
            "x": (0.0, 0.0),
            "y": (0.0, 0.0),
            "z": (0.0, 0.0),
            "roll": (0.0, 0.0),
            "pitch": (0.0, 0.0),
            "yaw": (0.0, 0.0),
        },
    }
    return True


def set_collision_enabled_recursive(stage, root_path: str, enabled: bool):
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return 0

    updated = 0
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collision_api = UsdPhysics.CollisionAPI(prim)
            collision_api.CreateCollisionEnabledAttr(bool(enabled))
            updated += 1
    return updated


def set_floor_collisions_enabled(stage, root_path: str, enabled: bool) -> int:
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return 0

    floor_tokens = ("floor", "ground", "tile")
    updated = 0
    for prim in Usd.PrimRange(root):
        name = prim.GetName().lower()
        path = prim.GetPath().pathString.lower()
        if not any(token in name or token in path for token in floor_tokens):
            continue
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collision_api = UsdPhysics.CollisionAPI(prim)
            collision_api.CreateCollisionEnabledAttr(bool(enabled))
            updated += 1
    return updated


def remap_visual_env_material_bindings(stage, root_path: str = "/World/visual_env") -> int:
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return 0

    rebound = 0
    old_prefix = "/World/Looks/"
    new_prefix = f"{root_path}/Looks/"

    for prim in Usd.PrimRange(root):
        binding_api = UsdShade.MaterialBindingAPI(prim)
        rel = binding_api.GetDirectBindingRel()
        if not rel:
            continue
        targets = rel.GetTargets()
        if not targets:
            continue
        new_targets = []
        changed = False
        for target in targets:
            target_str = target.pathString
            if target_str.startswith(old_prefix):
                new_targets.append(Sdf.Path(target_str.replace(old_prefix, new_prefix, 1)))
                changed = True
            else:
                new_targets.append(target)
        if changed:
            rel.SetTargets(new_targets)
            rebound += 1
    return rebound


def hide_imageables_recursive(stage, root_path: str) -> int:
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return 0

    hidden = 0
    for prim in Usd.PrimRange(root):
        try:
            imageable = UsdGeom.Imageable(prim)
            if imageable:
                imageable.MakeInvisible()
                hidden += 1
        except Exception:
            continue
    return hidden


def is_nonzero_command(cmd, eps: float = 1.0e-4) -> bool:
    return any(abs(float(v)) > eps for v in cmd)


class ABMovingObstacle:
    """A-B 구간을 왕복하는 단순 kinematic obstacle."""

    def __init__(
        self,
        stage,
        prim_path: str,
        point_a: tuple[float, float, float],
        point_b: tuple[float, float, float],
        speed_mps: float = 0.5,
        radius_m: float = 0.22,
        height_m: float = 1.2,
        visual_usd_path: str | None = None,
        visual_scale: float = 1.0,
        visual_z_offset: float = 0.0,
        visual_yaw_offset_deg: float = 0.0,
    ):
        self._stage = stage
        self._prim_path = prim_path
        self._point_a = np.asarray(point_a, dtype=float)
        self._point_b = np.asarray(point_b, dtype=float)
        self._speed_mps = max(float(speed_mps), 1.0e-3)
        self._segment = self._point_b - self._point_a
        self._segment_len = float(np.linalg.norm(self._segment))
        self._direction = self._segment / self._segment_len if self._segment_len > 1.0e-6 else np.array([0.0, 1.0, 0.0])
        self._period_s = (2.0 * self._segment_len) / self._speed_mps if self._segment_len > 1.0e-6 else 1.0
        self._start_time = time.time()
        self._visual_z_offset = float(visual_z_offset)
        self._visual_yaw_offset_deg = float(visual_yaw_offset_deg)

        root = UsdGeom.Xform.Define(stage, prim_path)
        root_prim = root.GetPrim()
        self._xform = UsdGeom.Xformable(root_prim)
        self._translate_op = self._xform.AddTranslateOp()
        self._rotate_op = self._xform.AddRotateXYZOp()

        self._visual_translate_op = None
        self._visual_rotate_op = None
        self._visual_scale_op = None
        visual_root = UsdGeom.Xform.Define(stage, f"{prim_path}/visual")
        visual_prim = visual_root.GetPrim()
        visual_xform = UsdGeom.Xformable(visual_prim)
        self._visual_translate_op = visual_xform.AddTranslateOp()
        self._visual_rotate_op = visual_xform.AddRotateXYZOp()
        self._visual_scale_op = visual_xform.AddScaleOp()
        self._visual_translate_op.Set(Gf.Vec3d(0.0, 0.0, self._visual_z_offset))
        self._visual_rotate_op.Set(Gf.Vec3f(0.0, 0.0, self._visual_yaw_offset_deg))
        self._visual_scale_op.Set(
            Gf.Vec3f(float(visual_scale), float(visual_scale), float(visual_scale))
        )

        if visual_usd_path:
            visual_prim.GetReferences().AddReference(str(visual_usd_path))
        else:
            visual_capsule = UsdGeom.Capsule.Define(stage, f"{prim_path}/visual/body")
            visual_capsule.CreateAxisAttr("Z")
            visual_capsule.CreateRadiusAttr(float(radius_m))
            visual_capsule.CreateHeightAttr(float(height_m))
            visual_capsule.CreateDisplayColorAttr([(0.12, 0.45, 0.90)])

        self.update(time.time())

    def update(self, now_s: float):
        if self._segment_len <= 1.0e-6:
            pos = self._point_a
            direction = self._direction
        else:
            phase = ((now_s - self._start_time) % self._period_s) / self._period_s
            travel = phase * 2.0
            if travel <= 1.0:
                alpha = travel
                direction = self._direction
            else:
                alpha = 2.0 - travel
                direction = -self._direction
            pos = (1.0 - alpha) * self._point_a + alpha * self._point_b

        yaw_deg = math.degrees(math.atan2(direction[1], direction[0]))
        self._translate_op.Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
        self._rotate_op.Set(Gf.Vec3f(0.0, 0.0, float(yaw_deg)))


class WasdKeyboard(Se2Keyboard):
    def _create_key_bindings(self):
        self._INPUT_KEY_MAPPING = {
            "W": np.asarray([1.0, 0.0, 0.0]) * self.v_x_sensitivity,
            "S": np.asarray([-1.0, 0.0, 0.0]) * self.v_x_sensitivity,
            "A": np.asarray([0.0, 1.0, 0.0]) * self.v_y_sensitivity,
            "D": np.asarray([0.0, -1.0, 0.0]) * self.v_y_sensitivity,
            "Q": np.asarray([0.0, 0.0, 1.0]) * self.omega_z_sensitivity,
            "E": np.asarray([0.0, 0.0, -1.0]) * self.omega_z_sensitivity,
            "K": np.asarray([0.0, 0.0, 0.0]),
        }

    def _poll_command(self):
        # carb에서 실제 키 상태를 매 프레임 직접 조회
        # 이벤트 누락(포커스 전환 시 KEY_RELEASE 미수신)으로 인한 stuck key 문제 방지
        import carb
        cmd = np.zeros(3)
        for key_name, delta in self._INPUT_KEY_MAPPING.items():
            key_enum = getattr(carb.input.KeyboardInput, key_name, None)
            if key_enum is not None and self._input.get_keyboard_value(self._keyboard, key_enum) > 0:
                cmd += delta
        self._base_command[:] = cmd
        return self._base_command

    def advance(self):
        self._poll_command()
        return torch.tensor(self._base_command, dtype=torch.float32, device=self._sim_device)


class ArrowKeyboard(Se2Keyboard):
    """방향키(↑↓←→) → /cmd_vel 테스트용. WASD와 키 충돌 없음."""
    def _create_key_bindings(self):
        self._INPUT_KEY_MAPPING = {
            "UP":    np.asarray([1.0, 0.0, 0.0]) * self.v_x_sensitivity,
            "DOWN":  np.asarray([-1.0, 0.0, 0.0]) * self.v_x_sensitivity,
            "LEFT":  np.asarray([0.0, 0.0, 1.0]) * self.omega_z_sensitivity,
            "RIGHT": np.asarray([0.0, 0.0, -1.0]) * self.omega_z_sensitivity,
        }

    def advance(self):
        # WasdKeyboard와 동일하게 실제 키 상태를 polling해서 방향키 release 누락을 막는다.
        return torch.tensor(
            WasdKeyboard._poll_command(self),
            dtype=torch.float32,
            device=self._sim_device,
        )


class CmdVelNode:
    """Isaac Sim 내장 rclpy로 /cmd_vel과 /sim/visible_objects를 처리.

    - 방향키 입력 → publish() → /cmd_vel 퍼블리시
    - /cmd_vel 수신 → get_latest() → 로봇 vel_command_b에 주입
    - 로봇 위치 기반 visible objects → publish_visible_objects()
    - 타임아웃(CMD_VEL_TIMEOUT 초) 동안 업데이트 없으면 자동 정지
    """

    CMD_VEL_TIMEOUT = 0.5  # 초

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String

        if not rclpy.ok():
            rclpy.init()

        self._rclpy = rclpy
        _lock = threading.Lock()
        _latest = [None]       # (vx, vy, omega) or None
        _last_recv_time = [0.0]
        _last_visible = [None]

        class _Node(Node):
            def __init__(self):
                super().__init__("go2_cmd_vel")
                self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
                self._visible_pub = self.create_publisher(String, "/sim/visible_objects", 10)
                self._named_place_pub = self.create_publisher(String, "/sim/named_place_poses", 10)
                self.create_subscription(Twist, "/cmd_vel", self._cb, 10)

            def _cb(self, msg):
                with _lock:
                    _latest[0] = (msg.linear.x, msg.linear.y, msg.angular.z)
                    _last_recv_time[0] = time.time()

            def publish(self, vx, vy, omega):
                msg = Twist()
                msg.linear.x = float(vx)
                msg.linear.y = float(vy)
                msg.angular.z = float(omega)
                self._pub.publish(msg)

            def publish_visible_objects(self, names):
                data = ",".join(names)
                _last_visible[0] = data
                msg = String()
                msg.data = data
                self._visible_pub.publish(msg)

            def publish_named_place_poses(self, pose_map):
                msg = String()
                msg.data = json.dumps(pose_map, sort_keys=True)
                self._named_place_pub.publish(msg)

        self._node = _Node()
        self._lock = _lock
        self._latest = _latest
        self._last_recv_time = _last_recv_time
        self._last_visible = _last_visible

        # spin_once 루프: 짧은 timeout으로 GIL을 자주 해제해 메인 루프 방해 최소화
        def _spin_loop():
            while rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.01)

        self._thread = threading.Thread(target=_spin_loop, daemon=True)
        self._thread.start()
        print("[INFO] /cmd_vel subscriber/publisher 시작 (방향키: ↑↓전후 ←→회전)")
        print("[INFO] /sim/visible_objects publisher 시작")
        print("[INFO] /sim/named_place_poses publisher 시작")

    def publish(self, vx: float, vy: float, omega: float):
        self._node.publish(vx, vy, omega)

    def publish_visible_objects(self, names):
        self._node.publish_visible_objects(sorted(set(names)))

    def publish_named_place_poses(self, pose_map):
        self._node.publish_named_place_poses(pose_map)

    def get_latest(self):
        """가장 최근 수신된 /cmd_vel 반환. 타임아웃 초과 시 None 반환 (정지)."""
        with self._lock:
            if self._latest[0] is None:
                return None
            if time.time() - self._last_recv_time[0] > self.CMD_VEL_TIMEOUT:
                return None
            return self._latest[0]

    def shutdown(self):
        self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


def setup_ros2_camera_graph(camera_prim_path: str):
    """숨겨진 뷰포트에서 렌더 프로덕트 생성 → OmniGraph ROS2 퍼블리시.

    공식 예제 방식: execution evaluator + SIMULATION pipeline + frameSkipCount
    → evaluate_sync 블로킹 없이 시뮬레이션 스텝과 자동 동기화
    """
    from omni.kit.viewport.utility import create_viewport_window

    # 숨겨진 뷰포트 생성 (메인 뷰포트에 영향 없음) - 320x240 저해상도
    vp_window = create_viewport_window(
        "ROS2_Camera", width=320, height=240, visible=False
    )
    vp_api = vp_window.viewport_api
    vp_api.set_active_camera(camera_prim_path)
    rp_path = vp_api.get_render_product_path()
    print(f"[INFO] 숨겨진 뷰포트 렌더 프로덕트: {rp_path}")

    # frameSkipCount: 퍼블리시Hz = simFPS / (skipCount + 1)
    # 시뮬레이션 ~30fps 기준 → skipCount=2 → ~10Hz 퍼블리시
    FRAME_SKIP = 2

    keys = og.Controller.Keys
    (ros_camera_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS2_Camera",
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "cameraHelperRgb.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "cameraHelperDepth.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "cameraHelperInfo.inputs:execIn"),
            ],
            keys.SET_VALUES: [
                ("cameraHelperRgb.inputs:renderProductPath", rp_path),
                ("cameraHelperRgb.inputs:frameId", "camera_optical_frame"),
                ("cameraHelperRgb.inputs:topicName", "camera/color/image_raw"),
                ("cameraHelperRgb.inputs:type", "rgb"),
                ("cameraHelperRgb.inputs:frameSkipCount", FRAME_SKIP),
                ("cameraHelperDepth.inputs:renderProductPath", rp_path),
                ("cameraHelperDepth.inputs:frameId", "camera_optical_frame"),
                ("cameraHelperDepth.inputs:topicName", "camera/depth/image_rect_raw"),
                ("cameraHelperDepth.inputs:type", "depth"),
                ("cameraHelperDepth.inputs:frameSkipCount", FRAME_SKIP),
                ("cameraHelperInfo.inputs:renderProductPath", rp_path),
                ("cameraHelperInfo.inputs:frameId", "camera_optical_frame"),
                ("cameraHelperInfo.inputs:topicName", "camera/camera_info"),
                ("cameraHelperInfo.inputs:frameSkipCount", FRAME_SKIP),
            ],
        },
    )
    print(f"[INFO] ROS2 카메라 퍼블리셔 설정 완료 (320x240, frameSkip={FRAME_SKIP})")

    # /clock 퍼블리시 (use_sim_time 지원)
    (clock_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS2_Clock",
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("readSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("publishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "publishClock.inputs:execIn"),
                ("readSimTime.outputs:simulationTime", "publishClock.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("publishClock.inputs:topicName", "/clock"),
            ],
        },
    )
    print("[INFO] ROS2 /clock 퍼블리셔 설정 완료")


def setup_odom_graph(chassis_prim_path: str):
    """Odometry + TF (odom → base_link) 퍼블리셔 설정.

    IsaacComputeOdometry가 prim에서 직접 position/orientation/velocity를 읽어
    같은 SIMULATION pipeline tick에서 퍼블리시 → 카메라와 완벽 동기화.
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": "/ROS2_Odom",
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("readSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("computeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("publishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("publishTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            keys.CONNECT: [
                # 실행 흐름: tick → computeOdom → publishOdom, publishTF
                ("OnPlaybackTick.outputs:tick", "computeOdom.inputs:execIn"),
                ("computeOdom.outputs:execOut", "publishOdom.inputs:execIn"),
                ("computeOdom.outputs:execOut", "publishTF.inputs:execIn"),
                # 타임스탬프
                ("readSimTime.outputs:simulationTime", "publishOdom.inputs:timeStamp"),
                ("readSimTime.outputs:simulationTime", "publishTF.inputs:timeStamp"),
                # computeOdom 출력 → publishOdom 입력
                ("computeOdom.outputs:position", "publishOdom.inputs:position"),
                ("computeOdom.outputs:orientation", "publishOdom.inputs:orientation"),
                ("computeOdom.outputs:linearVelocity", "publishOdom.inputs:linearVelocity"),
                ("computeOdom.outputs:angularVelocity", "publishOdom.inputs:angularVelocity"),
                # computeOdom 출력 → publishTF 입력
                ("computeOdom.outputs:position", "publishTF.inputs:translation"),
                ("computeOdom.outputs:orientation", "publishTF.inputs:rotation"),
            ],
            keys.SET_VALUES: [
                # Odometry 메시지 설정
                ("publishOdom.inputs:chassisFrameId", "base_link"),
                ("publishOdom.inputs:odomFrameId", "odom"),
                ("publishOdom.inputs:topicName", "/odom"),
                # TF: odom → base_link
                ("publishTF.inputs:parentFrameId", "odom"),
                ("publishTF.inputs:childFrameId", "base_link"),
                ("publishTF.inputs:topicName", "/tf"),
            ],
        },
    )

    # chassisPrim relationship 설정 (USD API 필요)
    import omni.usd
    from pxr import Sdf

    stage = omni.usd.get_context().get_stage()
    compute_prim = stage.GetPrimAtPath("/ROS2_Odom/computeOdom")
    compute_prim.GetRelationship("inputs:chassisPrim").SetTargets(
        [Sdf.Path(chassis_prim_path)]
    )
    print("[INFO] ROS2 Odometry + TF (odom → base_link) 퍼블리셔 설정 완료 (OmniGraph 동기화)")


def setup_imu_graph():
    """IMU 퍼블리셔 설정 (/imu/data).

    그래프만 생성하고, 실제 IMU 데이터는 메인 루프에서 주입합니다.
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": "/ROS2_IMU",
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("readSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("publishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "publishImu.inputs:execIn"),
                ("readSimTime.outputs:simulationTime", "publishImu.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("publishImu.inputs:frameId", "base_link"),
                ("publishImu.inputs:topicName", "/imu/data"),
            ],
        },
    )
    print("[INFO] ROS2 IMU 퍼블리셔 설정 완료 (/imu/data)")


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg, agent_cfg):
    # 3. Create Environment
    custom_env_cfg = MySlamEnvCfg()
    custom_env_cfg.scene.num_envs = args_cli.num_envs

    env = gym.make(args_cli.task, cfg=custom_env_cfg)
    env = RslRlVecEnvWrapper(env)

    # 4. Load Policy — Unitree RL Lab 최신 체크포인트 자동 탐색
    import glob, re as _re
    _log_dir = "/home/jnu/Unitree/unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity"
    _sessions = sorted(glob.glob(os.path.join(_log_dir, "*")))
    if not _sessions:
        raise FileNotFoundError(f"체크포인트 세션 없음: {_log_dir}")
    _latest_session = _sessions[-1]
    _pts = sorted(
        glob.glob(os.path.join(_latest_session, "model_*.pt")),
        key=lambda p: int(_re.search(r"model_(\d+)\.pt", p).group(1)),
    )
    if not _pts:
        raise FileNotFoundError(f"모델 파일 없음: {_latest_session}")
    resume_path = _pts[-1]

    print(f"[INFO] Loading policy from: {resume_path}")
    runner = OnPolicyRunner(
        env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device
    )
    # Unitree RL Lab은 actor(45-dim) / critic(60-dim) obs가 분리되어 학습됨.
    # 추론 시 critic 불필요 → critic 키를 제외한 actor 가중치만 로드.
    import torch as _torch
    _ckpt = _torch.load(resume_path, weights_only=False)
    _actor_state = {k: v for k, v in _ckpt["model_state_dict"].items() if not k.startswith("critic")}
    runner.alg.policy.load_state_dict(_actor_state, strict=False)
    print(f"[INFO] Policy (actor) weights loaded, critic skipped")
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    # 5. ROS2 OmniGraph 카메라 퍼블리셔 설정 (SIMULATION 파이프라인 - 자동 실행)
    cam_prim_path = "/World/envs/env_0/Robot/base/front_cam"
    try:
        setup_ros2_camera_graph(cam_prim_path)
    except Exception as e:
        print(f"[WARN] ROS2 bridge 설정 실패: {e}")

    # 5.5 Odometry + TF (odom → base_link) 퍼블리셔 설정 (OmniGraph 내 동기화)
    robot_base_prim = "/World/envs/env_0/Robot/base"
    try:
        setup_odom_graph(robot_base_prim)
    except Exception as e:
        print(f"[WARN] Odom 설정 실패: {e}")

    # 5.6 IMU 퍼블리셔 설정
    try:
        setup_imu_graph()
    except Exception as e:
        print(f"[WARN] IMU 설정 실패: {e}")

    # 6. Reset & Loop
    obs = env.get_observations()
    dt = env.unwrapped.step_dt
    keyboard = WasdKeyboard(
        Se2KeyboardCfg(
            v_x_sensitivity=1.0, v_y_sensitivity=1.0, omega_z_sensitivity=1.5
        )
    )
    arrow_keyboard = ArrowKeyboard(
        Se2KeyboardCfg(v_x_sensitivity=1.0, omega_z_sensitivity=1.5)
    )
    cmd_vel_node = CmdVelNode()
    visible_object_specs = load_visible_object_specs(VISIBLE_OBJECTS_CONFIG_PATH)
    named_place_specs = load_named_place_specs(NAMED_PLACES_CONFIG_PATH)
    _last_log_time = 0.0
    _last_arrow_publish = (0.0, 0.0, 0.0)

    import omni.usd
    from pxr import UsdGeom

    stage = omni.usd.get_context().get_stage()
    robot_asset = env.unwrapped.scene["robot"]
    visual_env_summary = None
    dynamic_person = None

    # 기본 ground의 시각 메쉬는 숨기고 물리 plane만 유지한다.
    try:
        hidden_ground = hide_imageables_recursive(stage, "/World/ground")
        print(f"[INFO] hidden /World/ground imageables: {hidden_ground}")
    except Exception:
        pass

    # visual_env를 실제 장애물로 쓸지, 렌더링 전용으로 쓸지 환경 변수로 제어한다.
    try:
        if VISUAL_ENV_COLLISIONS:
            print("[INFO] visual_env collisions enabled")
        else:
            disabled_collisions = set_collision_enabled_recursive(stage, "/World/visual_env", enabled=False)
            print(f"[INFO] visual_env collision disabled prims: {disabled_collisions}")
    except Exception as e:
        print(f"[WARN] failed to configure visual_env collisions: {e}")

    try:
        if VISUAL_ENV_COLLISIONS and VISUAL_ENV_DISABLE_FLOOR_COLLISIONS:
            disabled_floor = set_floor_collisions_enabled(stage, "/World/visual_env", enabled=False)
            print(f"[INFO] visual_env floor collisions disabled: {disabled_floor}")
    except Exception as e:
        print(f"[WARN] failed to disable visual_env floor collisions: {e}")

    try:
        rebound = remap_visual_env_material_bindings(stage, "/World/visual_env")
        if rebound:
            print(f"[INFO] visual_env material bindings remapped: {rebound}")
    except Exception as e:
        print(f"[WARN] failed to remap visual_env materials: {e}")

    try:
        if DYN_PERSON_ENABLED:
            dynamic_person = ABMovingObstacle(
                stage=stage,
                prim_path="/World/dynamic_obstacles/person_0",
                point_a=DYN_PERSON_A,
                point_b=DYN_PERSON_B,
                speed_mps=DYN_PERSON_SPEED_MPS,
                radius_m=DYN_PERSON_RADIUS_M,
                height_m=DYN_PERSON_HEIGHT_M,
                visual_usd_path=DYN_PERSON_USD_PATH or None,
                visual_scale=DYN_PERSON_VISUAL_SCALE,
                visual_z_offset=DYN_PERSON_VISUAL_Z_OFFSET,
                visual_yaw_offset_deg=DYN_PERSON_VISUAL_YAW_OFFSET_DEG,
            )
            print(
                "[INFO] dynamic person enabled: "
                f"A={DYN_PERSON_A} B={DYN_PERSON_B} "
                f"visual_usd={DYN_PERSON_USD_PATH or 'capsule_only'}"
            )
    except Exception as e:
        print(f"[WARN] failed to create dynamic person: {e}")

    for prim_path in (
        "/World/ground",
        "/World/visual_env",
        "/World/visual_env/WareHouse",
        "/World/visual_env/WareHouse/full_warehouse",
        "/World/visual_env/Environment",
        "/World/visual_env/Dynamics",
    ):
        print(f"[INFO] stage prim summary: {describe_stage_prim(stage, prim_path)}")

    try:
        anchor_path, visual_env_summary = find_camera_anchor(
            stage,
            [
                "/World/visual_env/WareHouse",
                "/World/visual_env/Environment",
                "/World/visual_env/Dynamics",
                "/World/visual_env",
            ],
        )
        if visual_env_summary is None:
            anchor_path, visual_env_summary = find_descendant_with_bbox(stage, "/World/visual_env")
        if visual_env_summary is not None:
            center = visual_env_summary["bbox_center"]
            if set_robot_spawn_to_xy(env, float(center[0]), float(center[1]), 0.0):
                obs, _ = env.reset()
                print(
                    "[INFO] robot spawn reset to visual_env center "
                    f"(x={center[0]:.3f}, y={center[1]:.3f})"
                )
        if visual_env_summary is not None:
            center = visual_env_summary["bbox_center"]
            cam_target = [center[0], center[1], center[2]]
            cam_eye = [center[0], center[1], center[2] + 15.0]
            print(f"[INFO] startup viewport set to fixed top-down map view at {anchor_path}")
        else:
            robot_root_pos = robot_asset.data.root_pos_w[0].detach().cpu().tolist()
            cam_target = [float(robot_root_pos[0]), float(robot_root_pos[1]), 0.0]
            cam_eye = [cam_target[0], cam_target[1], 15.0]
            print("[INFO] startup viewport set to top-down fallback view")
        env.unwrapped.sim.set_camera_view(eye=cam_eye, target=cam_target)
        print(
            "[INFO] startup viewport set "
            f"(eye={cam_eye}, target={cam_target})"
        )
    except Exception as e:
        try:
            env.unwrapped.sim.set_camera_view(eye=[0.0, 0.0, 18.0], target=[0.0, 0.0, 0.0])
            print(f"[WARN] robot-based startup viewport failed, fallback to origin view: {e}")
        except Exception as inner_e:
            print(f"[WARN] failed to set startup viewport: {inner_e}")

    def get_robot_xy():
        root_pos = robot_asset.data.root_pos_w[0, :2].detach().cpu().numpy()
        return float(root_pos[0]), float(root_pos[1])

    # 명령 manager 미리 캐싱
    cmd_term = None
    if hasattr(env.unwrapped, "command_manager"):
        cmd_term = env.unwrapped.command_manager.get_term("base_velocity")

    # IMU OmniGraph 속성 경로 헬퍼 (odom/TF는 OmniGraph 내부에서 자동 동기화)
    def _imu_attr(name):
        return og.Controller.attribute(f"/ROS2_IMU/publishImu.inputs:{name}")

    while simulation_app.is_running():
        start_time = time.time()
        now = time.time()
        if dynamic_person is not None:
            dynamic_person.update(now)
        vel_cmd = keyboard.advance()  # WASD: 로봇 직접 제어 (기존 그대로)

        # [테스트] 방향키 → /cmd_vel 퍼블리시 (WASD와 키 충돌 없음)
        arrow_vel = arrow_keyboard.advance()
        arrow_tuple = (float(arrow_vel[0]), float(arrow_vel[1]), float(arrow_vel[2]))
        if any(v != 0.0 for v in arrow_tuple) or arrow_tuple != _last_arrow_publish:
            cmd_vel_node.publish(*arrow_tuple)
            _last_arrow_publish = arrow_tuple

        # /cmd_vel 수신값 우선 적용, 없으면 WASD 폴백
        received = cmd_vel_node.get_latest()
        has_wasd_input = is_nonzero_command(vel_cmd)
        has_received_cmd = received is not None and is_nonzero_command(received)
        if cmd_term is not None:
            if has_wasd_input:
                cmd_term.vel_command_b[0, 0] = vel_cmd[0]
                cmd_term.vel_command_b[0, 1] = vel_cmd[1]
                cmd_term.vel_command_b[0, 2] = vel_cmd[2]
                if now - _last_log_time > 1.0:
                    print(
                        "[WASD_DEBUG] "
                        f"key=({float(vel_cmd[0]):.2f}, {float(vel_cmd[1]):.2f}, {float(vel_cmd[2]):.2f}) "
                        f"cmd=({float(cmd_term.vel_command_b[0, 0]):.2f}, "
                        f"{float(cmd_term.vel_command_b[0, 1]):.2f}, "
                        f"{float(cmd_term.vel_command_b[0, 2]):.2f})"
                    )
                    _last_log_time = now
            elif has_received_cmd:
                # Nav2 (또는 방향키 테스트) cmd_vel → 로봇 직접 제어
                cmd_term.vel_command_b[0, 0] = received[0]
                cmd_term.vel_command_b[0, 1] = received[1]
                cmd_term.vel_command_b[0, 2] = received[2]
                if now - _last_log_time > 1.0:
                    print(f"[CMD_VEL] vx={received[0]:.2f}  vy={received[1]:.2f}  omega={received[2]:.2f}")
                    _last_log_time = now
            else:
                # 유효한 cmd_vel이 없으면 정지 상태 유지
                cmd_term.vel_command_b[0, 0] = vel_cmd[0]
                cmd_term.vel_command_b[0, 1] = vel_cmd[1]
                cmd_term.vel_command_b[0, 2] = vel_cmd[2]

        # IMU 데이터 사전 주입 (env.step 내 SIMULATION pipeline에서 퍼블리시됨)
        try:
            imu = env.unwrapped.scene["imu_sensor"]
            imu_ang_vel = imu.data.ang_vel_b[0].cpu().numpy()
            imu_lin_acc = imu.data.lin_acc_b[0].cpu().numpy()
            imu_quat_wxyz = imu.data.quat_w[0].cpu().numpy()
            # Isaac Lab WXYZ → OmniGraph XYZW (IJKR)
            imu_quat_xyzw = [imu_quat_wxyz[1], imu_quat_wxyz[2], imu_quat_wxyz[3], imu_quat_wxyz[0]]

            og.Controller.set(_imu_attr("angularVelocity"), imu_ang_vel.tolist())
            og.Controller.set(_imu_attr("linearAcceleration"), imu_lin_acc.tolist())
            og.Controller.set(_imu_attr("orientation"), imu_quat_xyzw)
        except Exception:
            pass  # 초기 프레임에서 데이터 없을 수 있음

        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)
            # env.step() 내부에서 SIMULATION pipeline 실행:
            # - IsaacComputeOdometry가 prim에서 직접 pos/quat/vel 읽기
            # - ROS2PublishOdometry + ROS2PublishRawTransformTree 퍼블리시
            # - 카메라 렌더 + 퍼블리시
            # → 모두 같은 tick에서 실행되어 완벽 동기화

        try:
            robot_x, robot_y = get_robot_xy()
            visible_names, visible_debug_rows = compute_visible_objects(robot_x, robot_y, visible_object_specs, stage)
            named_place_poses = compute_named_place_poses(named_place_specs, stage)
            cmd_vel_node.publish_visible_objects(visible_names)
            cmd_vel_node.publish_named_place_poses(named_place_poses)
            if now - _last_log_time > 1.0:
                print(f"[VISIBLE_DEBUG] robot_xy=({robot_x:.3f}, {robot_y:.3f}) visible={visible_names}")
                print('[OBJECT_DEBUG] ' + ' | '.join(visible_debug_rows))
                _last_log_time = now
        except Exception as e:
            if now - _last_log_time > 1.0:
                print(f"[WARN] visible object publish failed: {e}")
                _last_log_time = now

        if args_cli.rt.lower() in ("true", "1", "yes"):
            sleep_time = dt - (time.time() - start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

    cmd_vel_node.shutdown()
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
