#!/usr/bin/env python3
"""发布脚本化周车控制信号，用于风险实验中的主动交互场景。"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, TextIO

import rclpy
from rclpy.node import Node
from vehicle_msgs.msg import ArenaInfoDynamic
from vehicle_msgs.msg import ControlSignal


@dataclass(frozen=True)
class ActorInitialState:
    """保存脚本控制车辆的初始状态，便于按相对时间生成开环轨迹。"""

    x: float
    y: float
    angle: float
    curvature: float
    velocity: float
    acceleration: float
    steer: float


@dataclass(frozen=True)
class ActorSegment:
    """单段脚本动作，使用局部车体坐标描述纵向推进和横向偏移。"""

    start: float
    end: float
    lon_acc: float
    lateral_offset: float
    final_heading_delta: float


@dataclass(frozen=True)
class IdmFollowConfig:
    """IDM 纵向跟车参数，用于生成可复现的闭环周车反应。"""

    target_vehicle_id: int
    desired_velocity: float
    min_gap: float
    time_headway: float
    max_acc: float
    comfortable_brake: float
    delta: float


@dataclass(frozen=True)
class IdmTelemetry:
    """保存单次 IDM 计算的中间量，供实验 CSV 复现闭环交互过程。"""

    target_vehicle_id: int
    gap: float
    relative_velocity: float
    desired_gap: float
    acceleration: float


@dataclass(frozen=True)
class ScriptedActor:
    """单辆脚本化风险周车的完整控制配置。"""

    actor_id: int
    topic: str
    mode: str
    initial_state: ActorInitialState
    segments: List[ActorSegment]
    idm_follow: Optional[IdmFollowConfig]


TELEMETRY_FIELDS = [
    "stamp",
    "elapsed",
    "actor_id",
    "mode",
    "target_vehicle_id",
    "current_x",
    "current_y",
    "current_angle",
    "current_velocity",
    "current_acceleration",
    "script_lateral",
    "script_heading_delta",
    "gap",
    "relative_velocity",
    "desired_gap",
    "idm_acceleration",
    "command_x",
    "command_y",
    "command_angle",
    "command_velocity",
    "command_acceleration",
]


def parse_bool_param(value: object) -> bool:
    """兼容 ROS launch 字符串和原生 bool 参数，统一解析布尔开关。"""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def csv_float(value: Optional[float]) -> str:
    """把浮点数格式化为稳定 CSV 字段；None 表示本行无该物理量。"""
    if value is None:
        return ""
    return f"{value:.6f}"


def smoothstep(ratio: float) -> float:
    """三次平滑插值函数，避免横向偏移在段首段尾出现速度突变。"""
    clipped = min(1.0, max(0.0, ratio))
    return clipped * clipped * (3.0 - 2.0 * clipped)


def load_vehicle_initial_states(vehicle_set_path: Path) -> Dict[int, ActorInitialState]:
    """从 vehicle_set.json 读取车辆初始状态，供脚本 actor 作为轨迹起点。"""
    data = json.loads(vehicle_set_path.read_text(encoding="utf-8"))
    states: Dict[int, ActorInitialState] = {}
    for item in data["vehicles"]["info"]:
        # playground 车辆文件使用 init_state；保留 state 兼容将来的回放式配置。
        state = item.get("init_state", item.get("state"))
        if state is None:
            raise KeyError(f"vehicle {item.get('id')} has no init_state/state field")
        states[int(item["id"])] = ActorInitialState(
            x=float(state["x"]),
            y=float(state["y"]),
            angle=float(state["angle"]),
            curvature=float(state["curvature"]),
            velocity=float(state["velocity"]),
            acceleration=float(state["acceleration"]),
            steer=float(state["steer"]),
        )
    return states


def parse_segments(raw_segments: List[dict]) -> List[ActorSegment]:
    """解析并校验脚本动作段，保证时间区间单调且可执行。"""
    segments: List[ActorSegment] = []
    previous_end = 0.0
    for raw in raw_segments:
        start = float(raw["start"])
        end = float(raw["end"])
        if end <= start:
            raise ValueError(f"invalid actor segment time range: start={start}, end={end}")
        if start < previous_end:
            raise ValueError("actor segments must be sorted and non-overlapping")
        previous_end = end
        segments.append(
            ActorSegment(
                start=start,
                end=end,
                lon_acc=float(raw.get("lon_acc", 0.0)),
                lateral_offset=float(raw.get("lateral_offset", 0.0)),
                final_heading_delta=float(raw.get("final_heading_delta", 0.0)),
            )
        )
    return segments


def parse_idm_follow(raw_config: Optional[dict]) -> Optional[IdmFollowConfig]:
    """解析 IDM 跟车配置；未配置时返回 None，保持旧脚本兼容。"""
    if raw_config is None:
        return None
    return IdmFollowConfig(
        target_vehicle_id=int(raw_config["target_vehicle_id"]),
        desired_velocity=float(raw_config.get("desired_velocity", 12.0)),
        min_gap=float(raw_config.get("min_gap", 6.0)),
        time_headway=float(raw_config.get("time_headway", 1.2)),
        max_acc=float(raw_config.get("max_acc", 1.5)),
        comfortable_brake=float(raw_config.get("comfortable_brake", 2.0)),
        delta=float(raw_config.get("delta", 4.0)),
    )


class ScriptedRiskActorNode(Node):
    """按照 JSON 脚本向 /ctrl/agent_{id} 发布开环 ControlSignal。"""

    def __init__(self) -> None:
        super().__init__("scripted_risk_actor_node")
        self.declare_parameter("script_path", "")
        self.declare_parameter("vehicle_info_path", "")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("arena_info_dynamic_topic", "/arena_info_dynamic")
        self.declare_parameter("telemetry_csv_enabled", True)
        self.declare_parameter(
            "telemetry_csv_path", "/tmp/epsilon_scripted_risk_actor_telemetry.csv"
        )

        script_path = Path(self.get_parameter("script_path").value)
        vehicle_info_path = Path(self.get_parameter("vehicle_info_path").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        arena_info_dynamic_topic = str(self.get_parameter("arena_info_dynamic_topic").value)
        telemetry_csv_enabled = parse_bool_param(
            self.get_parameter("telemetry_csv_enabled").value
        )
        telemetry_csv_path = Path(str(self.get_parameter("telemetry_csv_path").value))
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        if not script_path.exists():
            raise FileNotFoundError(f"risk actor script not found: {script_path}")
        if not vehicle_info_path.exists():
            raise FileNotFoundError(f"vehicle_set.json not found: {vehicle_info_path}")

        initial_states = load_vehicle_initial_states(vehicle_info_path)
        self.actors_ = self.load_script(script_path, initial_states)
        self.publishers_ = {
            actor.actor_id: self.create_publisher(ControlSignal, actor.topic, 10)
            for actor in self.actors_
        }
        self.latest_vehicle_states_: Dict[int, ActorInitialState] = {}
        self.arena_info_dynamic_sub_ = self.create_subscription(
            ArenaInfoDynamic,
            arena_info_dynamic_topic,
            self.on_arena_info_dynamic,
            10,
        )
        self.last_publish_elapsed_: Dict[int, float] = {}
        self.last_script_lateral_: Dict[int, float] = {}
        self.last_script_heading_: Dict[int, float] = {}
        self.telemetry_csv_enabled_ = telemetry_csv_enabled
        self.telemetry_csv_path_ = telemetry_csv_path
        self.telemetry_csv_file_: Optional[TextIO] = None
        self.telemetry_csv_writer_: Optional[csv.DictWriter] = None
        self.open_telemetry_csv_if_needed()
        self.start_time_ = self.get_clock().now()
        self.timer_ = self.create_timer(1.0 / publish_rate_hz, self.on_timer)
        self.get_logger().info(
            f"loaded {len(self.actors_)} scripted risk actors from {script_path}"
        )
        if self.telemetry_csv_enabled_:
            self.get_logger().info(
                f"scripted risk actor telemetry csv: {self.telemetry_csv_path_}"
            )

    def open_telemetry_csv_if_needed(self) -> None:
        """按需打开 telemetry CSV，并立即写 header，保证 timeout 退出也能留下表头。"""
        if not self.telemetry_csv_enabled_:
            return
        if not str(self.telemetry_csv_path_):
            self.get_logger().warn("telemetry_csv_path is empty; telemetry disabled")
            self.telemetry_csv_enabled_ = False
            return
        self.telemetry_csv_path_.parent.mkdir(parents=True, exist_ok=True)
        self.telemetry_csv_file_ = self.telemetry_csv_path_.open(
            "w", newline="", encoding="utf-8"
        )
        self.telemetry_csv_writer_ = csv.DictWriter(
            self.telemetry_csv_file_, fieldnames=TELEMETRY_FIELDS
        )
        self.telemetry_csv_writer_.writeheader()
        self.telemetry_csv_file_.flush()

    def close_telemetry_csv(self) -> None:
        """关闭 telemetry CSV，配合批量实验的 timeout 尽量减少数据丢失。"""
        if self.telemetry_csv_file_ is None:
            return
        self.telemetry_csv_file_.flush()
        self.telemetry_csv_file_.close()
        self.telemetry_csv_file_ = None
        self.telemetry_csv_writer_ = None

    def load_script(
        self, script_path: Path, initial_states: Dict[int, ActorInitialState]
    ) -> List[ScriptedActor]:
        """读取脚本 JSON，并为未显式配置 topic 的车辆补齐 /ctrl/agent_{id}。"""
        data = json.loads(script_path.read_text(encoding="utf-8"))
        actors: List[ScriptedActor] = []
        for raw_actor in data.get("actors", []):
            actor_id = int(raw_actor["id"])
            if actor_id not in initial_states:
                raise ValueError(f"actor id {actor_id} does not exist in vehicle_set.json")
            mode = str(raw_actor.get("mode", "open_loop"))
            idm_follow = parse_idm_follow(raw_actor.get("idm_follow"))
            if mode == "idm_follow" and idm_follow is None:
                raise ValueError(f"actor {actor_id} uses idm_follow without idm_follow config")
            actors.append(
                ScriptedActor(
                    actor_id=actor_id,
                    topic=str(raw_actor.get("topic", f"/ctrl/agent_{actor_id}")),
                    mode=mode,
                    initial_state=initial_states[actor_id],
                    segments=parse_segments(raw_actor.get("segments", [])),
                    idm_follow=idm_follow,
                )
            )
        if not actors:
            raise ValueError("risk actor script must contain at least one actor")
        return actors

    def on_arena_info_dynamic(self, msg: ArenaInfoDynamic) -> None:
        """缓存最新车辆状态，供 IDM actor 计算相对距离和相对速度。"""
        latest_states: Dict[int, ActorInitialState] = {}
        for vehicle in msg.vehicle_set.vehicles:
            state = vehicle.state
            latest_states[int(vehicle.id.data)] = ActorInitialState(
                x=float(state.vec_position.x),
                y=float(state.vec_position.y),
                angle=float(state.angle),
                curvature=float(state.curvature),
                velocity=float(state.velocity),
                acceleration=float(state.acceleration),
                steer=float(state.steer),
            )
        self.latest_vehicle_states_ = latest_states

    def scripted_offsets(self, actor: ScriptedActor, elapsed: float) -> tuple[float, float, float, float, float]:
        """累计脚本段的纵向、横向、速度、加速度和航向偏移。"""
        initial = actor.initial_state
        longitudinal = 0.0
        lateral = 0.0
        velocity = initial.velocity
        acceleration = 0.0
        heading_delta = 0.0
        cursor_time = 0.0
        for segment in actor.segments:
            if elapsed <= segment.start:
                longitudinal += velocity * max(0.0, elapsed - cursor_time)
                break

            longitudinal += velocity * max(0.0, segment.start - cursor_time)
            duration = segment.end - segment.start
            local_t = min(max(0.0, elapsed - segment.start), duration)
            longitudinal += velocity * local_t + 0.5 * segment.lon_acc * local_t * local_t
            ratio = local_t / duration
            lateral += segment.lateral_offset * smoothstep(ratio)
            heading_delta += segment.final_heading_delta * smoothstep(ratio)
            acceleration = segment.lon_acc if elapsed <= segment.end else 0.0

            if elapsed <= segment.end:
                velocity = max(0.0, velocity + segment.lon_acc * local_t)
                break

            velocity = max(0.0, velocity + segment.lon_acc * duration)
            cursor_time = segment.end
        else:
            longitudinal += velocity * max(0.0, elapsed - cursor_time)
        return longitudinal, lateral, velocity, acceleration, heading_delta

    def compute_idm_telemetry(self, actor: ScriptedActor) -> Optional[IdmTelemetry]:
        """根据最新动态场景计算 IDM 纵向加速度和中间量；状态不足时返回 None。"""
        config = actor.idm_follow
        if config is None:
            return None
        ego_state = self.latest_vehicle_states_.get(actor.actor_id)
        target_state = self.latest_vehicle_states_.get(config.target_vehicle_id)
        if ego_state is None or target_state is None:
            return None

        heading_x = math.cos(ego_state.angle)
        heading_y = math.sin(ego_state.angle)
        dx = target_state.x - ego_state.x
        dy = target_state.y - ego_state.y
        gap = dx * heading_x + dy * heading_y
        if gap <= 0.1:
            return IdmTelemetry(
                target_vehicle_id=config.target_vehicle_id,
                gap=gap,
                relative_velocity=ego_state.velocity - target_state.velocity,
                desired_gap=config.min_gap,
                acceleration=-config.comfortable_brake,
            )

        relative_velocity = ego_state.velocity - target_state.velocity
        desired_velocity = max(0.1, config.desired_velocity)
        sqrt_term = 2.0 * math.sqrt(config.max_acc * config.comfortable_brake)
        desired_gap = config.min_gap + max(
            0.0,
            ego_state.velocity * config.time_headway
            + ego_state.velocity * relative_velocity / max(0.1, sqrt_term),
        )
        free_road_term = (ego_state.velocity / desired_velocity) ** config.delta
        interaction_term = (desired_gap / max(0.1, gap)) ** 2
        acc = config.max_acc * (1.0 - free_road_term - interaction_term)
        acceleration = min(config.max_acc, max(-config.comfortable_brake, acc))
        return IdmTelemetry(
            target_vehicle_id=config.target_vehicle_id,
            gap=gap,
            relative_velocity=relative_velocity,
            desired_gap=desired_gap,
            acceleration=acceleration,
        )

    def build_signal(
        self, actor: ScriptedActor, elapsed: float
    ) -> tuple[ControlSignal, Dict[str, str]]:
        """根据当前时间生成开环状态消息，交给 phy_simulator 直接覆盖周车状态。"""
        if actor.mode == "open_loop":
            return self.build_open_loop_signal(actor, elapsed)
        if actor.mode == "idm_follow":
            return self.build_idm_follow_signal(actor, elapsed)
        raise ValueError(f"unsupported scripted risk actor mode: {actor.mode}")

    def new_control_signal(self) -> ControlSignal:
        """创建带 map 坐标系和当前时间戳的开环控制消息。"""
        msg = ControlSignal()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.is_openloop.data = True
        return msg

    def telemetry_row(
        self,
        actor: ScriptedActor,
        elapsed: float,
        current: ActorInitialState,
        msg: ControlSignal,
        script_lateral: float,
        script_heading_delta: float,
        idm_telemetry: Optional[IdmTelemetry],
    ) -> Dict[str, str]:
        """把单次控制输出和闭环中间量整理成一行 CSV。"""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
        target_vehicle_id = ""
        if actor.idm_follow is not None:
            target_vehicle_id = str(actor.idm_follow.target_vehicle_id)
        if idm_telemetry is not None:
            target_vehicle_id = str(idm_telemetry.target_vehicle_id)
        return {
            "stamp": csv_float(stamp),
            "elapsed": csv_float(elapsed),
            "actor_id": str(actor.actor_id),
            "mode": actor.mode,
            "target_vehicle_id": target_vehicle_id,
            "current_x": csv_float(current.x),
            "current_y": csv_float(current.y),
            "current_angle": csv_float(current.angle),
            "current_velocity": csv_float(current.velocity),
            "current_acceleration": csv_float(current.acceleration),
            "script_lateral": csv_float(script_lateral),
            "script_heading_delta": csv_float(script_heading_delta),
            "gap": csv_float(None if idm_telemetry is None else idm_telemetry.gap),
            "relative_velocity": csv_float(
                None if idm_telemetry is None else idm_telemetry.relative_velocity
            ),
            "desired_gap": csv_float(
                None if idm_telemetry is None else idm_telemetry.desired_gap
            ),
            "idm_acceleration": csv_float(
                None if idm_telemetry is None else idm_telemetry.acceleration
            ),
            "command_x": csv_float(float(msg.state.vec_position.x)),
            "command_y": csv_float(float(msg.state.vec_position.y)),
            "command_angle": csv_float(float(msg.state.angle)),
            "command_velocity": csv_float(float(msg.state.velocity)),
            "command_acceleration": csv_float(float(msg.state.acceleration)),
        }

    def append_telemetry(self, row: Dict[str, str]) -> None:
        """写入一行 telemetry，并逐行 flush，适配短时 timeout 实验。"""
        if self.telemetry_csv_writer_ is None or self.telemetry_csv_file_ is None:
            return
        self.telemetry_csv_writer_.writerow(row)
        self.telemetry_csv_file_.flush()

    def build_open_loop_signal(
        self, actor: ScriptedActor, elapsed: float
    ) -> tuple[ControlSignal, Dict[str, str]]:
        """沿用 MVP-16 的开环脚本轨迹生成方式，保证旧场景完全兼容。"""
        initial = actor.initial_state
        msg = self.new_control_signal()
        longitudinal, lateral, velocity, acceleration, heading_delta = self.scripted_offsets(
            actor, elapsed
        )
        cos_yaw = math.cos(initial.angle)
        sin_yaw = math.sin(initial.angle)
        msg.state.vec_position.x = initial.x + longitudinal * cos_yaw - lateral * sin_yaw
        msg.state.vec_position.y = initial.y + longitudinal * sin_yaw + lateral * cos_yaw
        msg.state.vec_position.z = 0.0
        msg.state.angle = initial.angle + heading_delta
        msg.state.curvature = initial.curvature
        msg.state.velocity = velocity
        msg.state.acceleration = acceleration
        msg.state.steer = initial.steer
        telemetry = self.telemetry_row(
            actor, elapsed, initial, msg, lateral, heading_delta, None
        )
        return msg, telemetry

    def build_idm_follow_signal(
        self, actor: ScriptedActor, elapsed: float
    ) -> tuple[ControlSignal, Dict[str, str]]:
        """基于最新 arena_info_dynamic 做 IDM 纵向闭环，并叠加脚本横向偏移。"""
        current = self.latest_vehicle_states_.get(actor.actor_id, actor.initial_state)
        previous_elapsed = self.last_publish_elapsed_.get(actor.actor_id, elapsed)
        dt = max(1.0e-3, elapsed - previous_elapsed)
        _, lateral, _, scripted_acceleration, heading_delta = self.scripted_offsets(actor, elapsed)
        previous_lateral = self.last_script_lateral_.get(actor.actor_id, 0.0)
        previous_heading_delta = self.last_script_heading_.get(actor.actor_id, 0.0)
        lateral_delta = lateral - previous_lateral
        heading_delta_step = heading_delta - previous_heading_delta
        self.last_publish_elapsed_[actor.actor_id] = elapsed
        self.last_script_lateral_[actor.actor_id] = lateral
        self.last_script_heading_[actor.actor_id] = heading_delta

        idm_telemetry = self.compute_idm_telemetry(actor)
        acceleration = (
            scripted_acceleration if idm_telemetry is None else idm_telemetry.acceleration
        )
        velocity = max(0.0, current.velocity + acceleration * dt)
        average_velocity = 0.5 * (current.velocity + velocity)

        cos_yaw = math.cos(current.angle)
        sin_yaw = math.sin(current.angle)
        longitudinal_delta = average_velocity * dt

        msg = self.new_control_signal()
        msg.state.vec_position.x = (
            current.x + longitudinal_delta * cos_yaw - lateral_delta * sin_yaw
        )
        msg.state.vec_position.y = (
            current.y + longitudinal_delta * sin_yaw + lateral_delta * cos_yaw
        )
        msg.state.vec_position.z = 0.0
        msg.state.angle = current.angle + heading_delta_step
        msg.state.curvature = current.curvature
        msg.state.velocity = velocity
        msg.state.acceleration = acceleration
        msg.state.steer = current.steer
        telemetry = self.telemetry_row(
            actor, elapsed, current, msg, lateral, heading_delta, idm_telemetry
        )
        return msg, telemetry

    def on_timer(self) -> None:
        """定时发布所有脚本 actor 的开环控制信号。"""
        elapsed = (self.get_clock().now() - self.start_time_).nanoseconds * 1.0e-9
        for actor in self.actors_:
            msg, telemetry = self.build_signal(actor, elapsed)
            self.publishers_[actor.actor_id].publish(msg)
            self.append_telemetry(telemetry)


def main() -> None:
    """ROS2 节点入口。"""
    rclpy.init()
    node = ScriptedRiskActorNode()
    try:
        rclpy.spin(node)
    finally:
        node.close_telemetry_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
