#!/usr/bin/env python3
"""发布脚本化周车控制信号，用于风险实验中的主动交互场景。"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
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
class ScriptedActor:
    """单辆脚本化风险周车的完整控制配置。"""

    actor_id: int
    topic: str
    initial_state: ActorInitialState
    segments: List[ActorSegment]


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


class ScriptedRiskActorNode(Node):
    """按照 JSON 脚本向 /ctrl/agent_{id} 发布开环 ControlSignal。"""

    def __init__(self) -> None:
        super().__init__("scripted_risk_actor_node")
        self.declare_parameter("script_path", "")
        self.declare_parameter("vehicle_info_path", "")
        self.declare_parameter("publish_rate_hz", 50.0)

        script_path = Path(self.get_parameter("script_path").value)
        vehicle_info_path = Path(self.get_parameter("vehicle_info_path").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
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
        self.start_time_ = self.get_clock().now()
        self.timer_ = self.create_timer(1.0 / publish_rate_hz, self.on_timer)
        self.get_logger().info(
            f"loaded {len(self.actors_)} scripted risk actors from {script_path}"
        )

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
            actors.append(
                ScriptedActor(
                    actor_id=actor_id,
                    topic=str(raw_actor.get("topic", f"/ctrl/agent_{actor_id}")),
                    initial_state=initial_states[actor_id],
                    segments=parse_segments(raw_actor.get("segments", [])),
                )
            )
        if not actors:
            raise ValueError("risk actor script must contain at least one actor")
        return actors

    def build_signal(self, actor: ScriptedActor, elapsed: float) -> ControlSignal:
        """根据当前时间生成开环状态消息，交给 phy_simulator 直接覆盖周车状态。"""
        initial = actor.initial_state
        msg = ControlSignal()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.is_openloop.data = True

        # 默认沿初始航向行驶；脚本段会累计改变速度、横向偏移和航向。
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
        return msg

    def on_timer(self) -> None:
        """定时发布所有脚本 actor 的开环控制信号。"""
        elapsed = (self.get_clock().now() - self.start_time_).nanoseconds * 1.0e-9
        for actor in self.actors_:
            self.publishers_[actor.actor_id].publish(self.build_signal(actor, elapsed))


def main() -> None:
    """ROS2 节点入口。"""
    rclpy.init()
    node = ScriptedRiskActorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
