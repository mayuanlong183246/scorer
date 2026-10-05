#!/usr/bin/env python3
"""Text observer for sentry tactical scorer simulation."""

import os
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import rclpy
from auto_aim_interfaces.msg import Target
from geometry_msgs.msg import Point
from pb2025_sentry_decision_interfaces.msg import TacticalDecision
from pb2025_sentry_decision_interfaces.msg import TacticalExecution
from pb_rm_interfaces.msg import EventData, GameRobotHP, GameStatus, GroundRobotPosition
from pb_rm_interfaces.msg import RobotStatus
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class SentryTacticalTextObserver(Node):
    def __init__(self) -> None:
        super().__init__("sentry_tactical_text_observer")
        self.declare_parameter("team_color", "blue")
        self.declare_parameter("log_file", "auto")

        self.team_color = self.get_parameter("team_color").get_parameter_value().string_value
        self.log_file = self.resolve_log_file(
            self.get_parameter("log_file").get_parameter_value().string_value
        )

        self.decision: Optional[TacticalDecision] = None
        self.game_status: Optional[GameStatus] = None
        self.robot_status: Optional[RobotStatus] = None
        self.event_data: Optional[EventData] = None
        self.all_robot_hp: Optional[GameRobotHP] = None
        self.ground_positions: Optional[GroundRobotPosition] = None
        self.target: Optional[Target] = None
        self.simulation_state = "scenario=unknown phase=UNKNOWN event=unknown"
        self.last_decision_signature = ""
        self.last_execution_signature = ""
        self.last_world_signature = ""
        self.execution = None
        self.navigation = "waiting"
        self.create_timer(1.0, self.snapshot)
        self.create_subscription(String, "/tactical_navigation_status", self.on_navigation, 10)
        self.sim_fields = {}
        self.log_handle = open(self.log_file, "a", encoding="utf-8", buffering=1)

        self.create_subscription(TacticalDecision, "/tactical_decision", self.on_decision, 10)
        self.create_subscription(TacticalExecution, "/tactical_execution", self.on_execution, 10)
        self.create_subscription(GameStatus, "referee/game_status", self.on_game_status, 10)
        self.create_subscription(RobotStatus, "referee/robot_status", self.on_robot_status, 10)
        self.create_subscription(EventData, "referee/event_data", self.on_event_data, 10)
        self.create_subscription(GameRobotHP, "referee/all_robot_hp", self.on_all_robot_hp, 10)
        self.create_subscription(
            GroundRobotPosition,
            "referee/ground_robot_position",
            self.on_ground_positions,
            10,
        )
        self.create_subscription(Target, "tracker/target", self.on_target, qos_profile_sensor_data)
        self.create_subscription(
            String, "/referee/simulation_state", self.on_simulation_state, 10
        )

        self.text_pub = self.create_publisher(String, "/tactical_debug_text", 10)
        self.get_logger().info(
            "Started sentry tactical text observer: "
            f"team={self.team_color} "
            f"log_file={self.log_file}"
        )

    def resolve_log_file(self, configured: str) -> str:
        if configured and configured != "auto":
            path = configured
        else:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = str(self.workspace_text_dir() / f"sentry_match_{stamp}.txt")
        if not os.path.isabs(path):
            path = str(Path.cwd() / path)
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        return path

    @staticmethod
    def workspace_text_dir() -> Path:
        configured_workspace = os.environ.get("RMUC_WORKSPACE")
        if configured_workspace:
            workspace_root = Path(configured_workspace).expanduser().resolve()
        else:
            current = Path.cwd().resolve()
            workspace_root = current
            for candidate in (current, *current.parents):
                if (candidate / "src/pb2025_sentry_behavior").is_dir():
                    workspace_root = candidate
                    break

        text_dir = workspace_root / "text"
        text_dir.mkdir(parents=True, exist_ok=True)
        return text_dir

    def write_line(self, line: str) -> None:
        self.log_handle.write(line.rstrip() + "\n")
        self.log_handle.flush()

    def current_time(self) -> str:
        return datetime.now().strftime("%H:%M:%S")

    def simulation_value(self, key: str, default: str = "-") -> str:
        return self.sim_fields.get(key, default)

    def on_decision(self, msg: TacticalDecision) -> None:
        decision_signature = "|".join(
            [
                msg.action_name,
                str(msg.emergency),
                msg.target_id,
            ]
        )
        self.decision = msg
        if decision_signature != self.last_decision_signature:
            self.last_decision_signature = decision_signature
            reasons = ";".join(msg.top_reasons + msg.rejected_actions) or "-"
            self.write_line(
                f"[DECISION] time={self.current_time()} id={msg.decision_id} "
                f"world={msg.world_state_id} action={msg.action_name} score={msg.score:.2f} "
                f"emergency={str(msg.emergency).lower()} target={msg.target_id or '-'} "
                f"reason={reasons}"
            )

    def on_execution(self, msg: TacticalExecution) -> None:
        self.execution = msg
        signature = f"{msg.action}|{msg.state}|{msg.reason}"
        if signature != self.last_execution_signature:
            self.last_execution_signature = signature
            action_names = {
                0: "HOLD_SAFE", 1: "COVER_TEAMMATE", 2: "ATTACK_OUTPOST",
                3: "RESUPPLY_HOME", 4: "DEFEND_HOME", 5: "CAPTURE_POINT",
                6: "GATHER_INFO",
            }
            self.write_line(
                f"[EXECUTION] time={self.current_time()} decision={msg.decision_id} "
                f"world={msg.world_state_id} action={action_names.get(int(msg.action), int(msg.action))} "
                f"state={msg.state} reason={msg.reason or '-'}"
            )

    def on_game_status(self, msg: GameStatus) -> None:
        if self.game_status is not None and (
            self.game_status.game_progress == msg.game_progress
        ):
            return
        self.game_status = msg

    def on_robot_status(self, msg: RobotStatus) -> None:
        self.robot_status = msg

    def on_event_data(self, msg: EventData) -> None:
        if self.event_data is not None and (
            self.event_data.base_hp == msg.base_hp
            and self.event_data.fortress_gain_zone == msg.fortress_gain_zone
            and self.event_data.center_gain_zone == msg.center_gain_zone
        ):
            return
        self.event_data = msg

    def on_all_robot_hp(self, msg: GameRobotHP) -> None:
        self.all_robot_hp = msg

    def on_ground_positions(self, msg: GroundRobotPosition) -> None:
        self.ground_positions = msg

    def on_target(self, msg: Target) -> None:
        self.target = msg

    def on_simulation_state(self, msg: String) -> None:
        self.simulation_state = msg.data
        fields = dict(
            part.split("=", 1) for part in msg.data.split() if "=" in part
        )
        self.sim_fields = fields
        signature = "|".join(
            fields.get(key, "-") for key in ("phase", "event")
        )
        if signature == self.last_world_signature:
            return
        self.last_world_signature = signature
        target = self.target.id if self.target and self.target.tracking else "-"
        self.write_line(
            f"[WORLD] time={self.current_time()} sim={fields.get('sim_time', '-')} "
            f"world={fields.get('world_state_id', '-')} phase={fields.get('phase', '-')} "
            f"event={fields.get('event', '-')} hp={self.robot_status.current_hp if self.robot_status else '-'} "
            f"ammo={self.robot_status.projectile_allowance_17mm if self.robot_status else '-'} "
            f"heat={self.robot_status.shooter_17mm_1_barrel_heat if self.robot_status else '-'} "
            f"base={self.event_data.base_hp if self.event_data else '-'} "
            f"outpost={self.outpost_hp()} target={target}"
        )

    def on_navigation(self, message):
        self.navigation = message.data
        self.write_line(f"[NAVIGATION] time={self.current_time()} {message.data}")

    def snapshot(self):
        decision = self.decision.action_name if self.decision else "waiting"
        execution = (f"{self.execution.action}/{self.execution.state}/{self.execution.reason}"
                     if self.execution else "waiting")
        line = (f"[STATUS] time={self.current_time()} {self.simulation_state} "
                f"{self.robot_line()} {self.field_line()} "
                f"recommended={decision} executing={execution} navigation={self.navigation}")
        self.write_line(line)
        self.get_logger().info(line)
        self.text_pub.publish(String(data=line))

    def robot_line(self) -> str:
        if self.robot_status is None:
            return "robot: -"
        pos = self.robot_status.robot_pos.position
        return (
            "robot: "
            f"hp={self.robot_status.current_hp} "
            f"ammo={self.robot_status.projectile_allowance_17mm} "
            f"heat={self.robot_status.shooter_17mm_1_barrel_heat} "
            f"pos={self.point_text(pos)}"
        )

    def field_line(self) -> str:
        game = self.game_progress_text()
        if self.event_data is None:
            return f"field: game={game}"
        return (
            "field: "
            f"game={game} "
            f"base_hp={self.event_data.base_hp} "
            f"fortress={self.occupation_text(self.event_data.fortress_gain_zone)} "
            f"center={self.occupation_text(self.event_data.center_gain_zone)} "
            f"outpost_hp={self.outpost_hp()}"
        )

    def team_line(self) -> str:
        if self.ground_positions is None or self.all_robot_hp is None:
            return "team: -"
        teammates = [
            ("hero", 1, self.ground_positions.hero_position),
            ("engineer", 2, self.ground_positions.engineer_position),
            ("standard3", 3, self.ground_positions.standard_3_position),
            ("standard4", 4, self.ground_positions.standard_4_position),
        ]
        fields = [
            f"{name}={self.robot_hp(robot_id)}@{self.point_text(point)}"
            for name, robot_id, point in teammates
        ]
        return "team: " + " ".join(fields)

    def target_line(self) -> str:
        if self.target is None:
            return "target: -"
        return (
            "target: "
            f"tracking={str(self.target.tracking).lower()} "
            f"id={self.target.id or '-'} "
            f"pos={self.point_text(self.target.position)}"
        )

    def game_progress_text(self) -> str:
        if self.game_status is None:
            return "-"
        mapping = {
            self.game_status.NOT_START: "NOT_START",
            self.game_status.PREPARATION: "PREPARATION",
            self.game_status.SELF_CHECKING: "SELF_CHECKING",
            self.game_status.COUNT_DOWN: "COUNT_DOWN",
            self.game_status.RUNNING: "RUNNING",
            self.game_status.GAME_OVER: "GAME_OVER",
        }
        return mapping.get(self.game_status.game_progress, str(self.game_status.game_progress))

    @staticmethod
    def occupation_text(value: int) -> str:
        mapping = {
            EventData.UNOCCUPIED: "UNOCCUPIED",
            EventData.OCCUPIED_FRIEND: "FRIEND",
            EventData.OCCUPIED_ENEMY: "ENEMY",
            EventData.OCCUPIED_BOTH: "BOTH",
        }
        return mapping.get(value, str(value))

    def robot_hp(self, robot_id: int) -> int:
        if self.all_robot_hp is None:
            return 0
        red = self.team_color == "red"
        if robot_id == 1:
            return self.all_robot_hp.red_1_robot_hp if red else self.all_robot_hp.blue_1_robot_hp
        if robot_id == 2:
            return self.all_robot_hp.red_2_robot_hp if red else self.all_robot_hp.blue_2_robot_hp
        if robot_id == 3:
            return self.all_robot_hp.red_3_robot_hp if red else self.all_robot_hp.blue_3_robot_hp
        if robot_id == 4:
            return self.all_robot_hp.red_4_robot_hp if red else self.all_robot_hp.blue_4_robot_hp
        if robot_id == 7:
            return self.all_robot_hp.red_7_robot_hp if red else self.all_robot_hp.blue_7_robot_hp
        return 0

    def outpost_hp(self) -> int:
        if self.all_robot_hp is None:
            return 0
        return (
            self.all_robot_hp.blue_outpost_hp
            if self.team_color == "red"
            else self.all_robot_hp.red_outpost_hp
        )

    @staticmethod
    def point_text(point: Point) -> str:
        return f"({point.x:.1f},{point.y:.1f})"

    def close_log(self) -> None:
        if not self.log_handle.closed:
            self.log_handle.flush()
            self.log_handle.close()


def main(args: List[str] = None) -> None:
    rclpy.init(args=args)
    node = SentryTacticalTextObserver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close_log()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
