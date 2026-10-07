#!/usr/bin/env python3
"""Publish RoboMaster-like referee data for sentry tactical scorer testing."""

import math
import random
from dataclasses import dataclass
from typing import Callable, Dict, List, Tuple

import rclpy
from auto_aim_interfaces.msg import Target
from geometry_msgs.msg import Point
from pb_rm_interfaces.msg import EventData, GameRobotHP, GameStatus, GroundRobotPosition
from pb_rm_interfaces.msg import RfidStatus, RobotStatus
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from std_msgs.msg import String


SCENARIOS = [
    "hold_safe",
    "resupply_home",
    "defend_base",
    "defend_fortress",
    "cover_teammate",
    "attack_outpost",
    "capture_point",
    "gather_info",
    "resupply_complete",
    "outpost_destroyed",
    "game_over",
]


@dataclass
class ScenarioState:
    name: str
    game_status: GameStatus
    robot_status: RobotStatus
    event_data: EventData
    rfid_status: RfidStatus
    all_robot_hp: GameRobotHP
    ground_positions: GroundRobotPosition
    target: Target
    event_name: str = ""


def make_point(x: float, y: float, z: float = 0.0) -> Point:
    point = Point()
    point.x = x
    point.y = y
    point.z = z
    return point


class SentryRefereeSimulator(Node):
    def __init__(self) -> None:
        super().__init__("sentry_referee_simulator")
        self.declare_parameter("scenario", "hold_safe")
        self.declare_parameter("scenario_period_sec", 6.0)
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("team_color", "blue")
        self.declare_parameter("match_duration_sec", 180.0)
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("random_seed", 20260921)
        self.declare_parameter("simulation_log_period_sec", 2.0)
        self.declare_parameter("gazebo_navigation", False)

        self.declare_parameter("publish_target", True,
                               ParameterDescriptor(read_only=True))
        self.declare_parameter("paused_topics", "")
        # -1 means use the scenario value; all overrides are resettable at runtime.
        for name in ("game_progress", "remaining_time", "hp", "ammo", "heat",
                     "base_hp", "outpost_hp", "center_state", "fortress_state", "teammate_hp"):
            self.declare_parameter(name, -1)
        self.declare_parameter("teammate_x", 4.0)
        self.declare_parameter("teammate_y", 1.8)
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.validate_values({name: self.get_parameter(name).value for name in
                              self._parameters if name != "paused_topics"})

        self.scenario = self.get_parameter("scenario").get_parameter_value().string_value
        self.scenario_period_sec = (
            self.get_parameter("scenario_period_sec").get_parameter_value().double_value
        )
        self.publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.team_color = self.get_parameter("team_color").get_parameter_value().string_value
        self.match_duration_sec = max(
            self.get_parameter("match_duration_sec").get_parameter_value().double_value, 1.0
        )
        self.speed = max(self.get_parameter("speed").get_parameter_value().double_value, 0.01)
        self.random_seed = self.get_parameter("random_seed").get_parameter_value().integer_value
        self.simulation_log_period_sec = max(
            self.get_parameter("simulation_log_period_sec")
            .get_parameter_value()
            .double_value,
            0.1,
        )
        self.simulation_step_sec = 0.1
        self.match_state = None
        self.last_simulated_time = 0.0
        self.world_state_id = 0
        self.sim_rng = random.Random(self.random_seed)

        if self.scenario not in {"auto", "match"} and self.scenario not in SCENARIOS:
            raise ValueError(
                f"Unknown scenario '{self.scenario}'. Valid: match, auto, {SCENARIOS}"
            )

        qos_depth = 10
        self.game_status_pub = self.create_publisher(GameStatus, "referee/game_status", qos_depth)
        self.robot_status_pub = self.create_publisher(
            RobotStatus, "referee/robot_status", qos_depth
        )
        self.event_data_pub = self.create_publisher(EventData, "referee/event_data", qos_depth)
        self.rfid_status_pub = self.create_publisher(RfidStatus, "referee/rfid_status", qos_depth)
        self.all_robot_hp_pub = self.create_publisher(
            GameRobotHP, "referee/all_robot_hp", qos_depth
        )
        self.ground_position_pub = self.create_publisher(
            GroundRobotPosition, "referee/ground_robot_position", qos_depth
        )
        self.target_pub = (self.create_publisher(Target, "tracker/target", qos_depth)
                           if self.get_parameter("publish_target").value else None)
        self.simulation_state_pub = self.create_publisher(
            String, "/referee/simulation_state", qos_depth
        )

        self.scenario_start_time = self.get_clock().now()
        self.current_scenario_name = ""
        self.last_output_signature: Tuple[str, str, int, int] = ("", "", -1, -1)
        self.last_output_time = -self.simulation_log_period_sec

        period = 1.0 / max(self.publish_rate, 0.1)
        self.timer = self.create_timer(period, self.on_timer)
        self.publisher_check_timer = self.create_timer(2.0, self.check_referee_publishers)
        self.publisher_check_done = False
        self.get_logger().info(
            f"Started sentry referee simulator: scenario={self.scenario} "
            f"period={self.scenario_period_sec:.1f}s rate={self.publish_rate:.1f}Hz "
            f"team={self.team_color} duration={self.match_duration_sec:.1f}s "
            f"speed={self.speed:.2f} seed={self.random_seed}"
        )

    def check_referee_publishers(self) -> None:
        """Warn when another node publishes the same standard referee topics."""
        topics = (
            "game_status", "robot_status", "event_data", "rfid_status",
            "all_robot_hp", "ground_robot_position",
        )
        conflicts = []
        for topic in topics:
            count = len(self.get_publishers_info_by_topic("/referee/" + topic))
            if count > 1:
                conflicts.append(f"/referee/{topic} ({count} publishers)")
        if conflicts:
            self.get_logger().warning(
                "Multiple referee publishers detected; stop either the real referee "
                "or this simulator: " + ", ".join(conflicts))
            self.publisher_check_done = True
        elif not self.publisher_check_done:
            self.get_logger().info(
                "Publishing standard /referee topics; no competing referee publisher detected")
            self.publisher_check_done = True

    def on_timer(self) -> None:
        # Allow ros2 param set to switch navigation test scenarios at runtime.
        scenario = self.get_parameter("scenario").value
        if scenario != self.scenario:
            self.scenario = scenario
            self.scenario_start_time = self.get_clock().now()
            self.match_state = None
            self.last_simulated_time = 0.0
            self.sim_rng = random.Random(self.random_seed)
            self.last_output_signature = ()
        scenario_name = self.resolve_scenario()
        state = self.build_scenario(scenario_name)
        if self.get_parameter("gazebo_navigation").value:
            state.ground_positions.standard_3_position = make_point(5.0, 7.0)
            state.target.position = make_point(9.0, 5.0)
        self.apply_overrides(state)
        self.world_state_id += 1
        now_msg = self.get_clock().now().to_msg()
        state.target.header.stamp = now_msg
        state.target.header.frame_id = "map"

        paused = self.get_parameter("paused_topics").value.split(",")
        for name, publisher, message in (
            ("game_status", self.game_status_pub, state.game_status),
            ("robot_status", self.robot_status_pub, state.robot_status),
            ("event_data", self.event_data_pub, state.event_data),
            ("rfid_status", self.rfid_status_pub, state.rfid_status),
            ("all_robot_hp", self.all_robot_hp_pub, state.all_robot_hp),
            ("ground_robot_position", self.ground_position_pub, state.ground_positions),
            ("target", self.target_pub, state.target),
        ):
            if publisher is not None and name not in paused:
                publisher.publish(message)

        simulation_state = self.simulation_state_text(state)
        self.simulation_state_pub.publish(String(data=simulation_state))
        self.log_state_if_needed(state, simulation_state)
    def validate_values(self, values):
        if values.get("scenario", "hold_safe") not in SCENARIOS + ["auto", "match"]:
            raise ValueError("Unknown scenario")
        if values.get("team_color", "blue") not in ("red", "blue"):
            raise ValueError("team_color must be red or blue")
        for name in ("publish_rate", "scenario_period_sec", "match_duration_sec",
                     "speed", "simulation_log_period_sec"):
            if name in values and (not math.isfinite(values[name]) or values[name] <= 0):
                raise ValueError(f"{name} must be positive and finite")
        limits = {"game_progress": 5, "remaining_time": 65535, "hp": 400,
                  "ammo": 65535, "heat": 400, "base_hp": 65535,
                  "outpost_hp": 65535, "center_state": 3, "fortress_state": 3,
                  "teammate_hp": 400}
        for name, maximum in limits.items():
            if name in values and not -1 <= values[name] <= maximum:
                raise ValueError(f"{name} must be -1 or 0..{maximum}")
        for name in ("teammate_x", "teammate_y"):
            if name in values and not math.isfinite(values[name]):
                raise ValueError(f"{name} must be finite")
        known = {"game_status", "robot_status", "event_data", "rfid_status",
                 "all_robot_hp", "ground_robot_position", "target"}
        if not set(filter(None, values.get("paused_topics", "").split(","))).issubset(known):
            raise ValueError("Unknown paused topic (use short topic names)")

    def validate_parameters(self, parameters):
        try:
            values = {p.name: p.value for p in parameters}
            # Timing/topology changes require restarting the publisher. Reject rather
            # than accepting a value that the running node does not actually use.
            fixed = {"team_color", "publish_rate", "scenario_period_sec", "speed",
                     "match_duration_sec", "random_seed", "simulation_log_period_sec",
                     "gazebo_navigation", "use_sim_time"}
            if fixed.intersection(values):
                raise ValueError("Timing, team and topology parameters are startup-only")
            self.validate_values(values)
            return SetParametersResult(successful=True)
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))

    def apply_overrides(self, state):
        fields = {"game_progress": (state.game_status, "game_progress"),
                  "remaining_time": (state.game_status, "stage_remain_time"),
                  "hp": (state.robot_status, "current_hp"),
                  "ammo": (state.robot_status, "projectile_allowance_17mm"),
                  "heat": (state.robot_status, "shooter_17mm_1_barrel_heat"),
                  "base_hp": (state.event_data, "base_hp"),
                  "center_state": (state.event_data, "center_gain_zone"),
                  "fortress_state": (state.event_data, "fortress_gain_zone")}
        for name, (message, field) in fields.items():
            value = self.get_parameter(name).value
            if value >= 0:
                setattr(message, field, value)
        for name, setter in (("outpost_hp", self.set_outpost_hp),
                             ("teammate_hp", lambda state, hp: self.set_teammate_hp(state, 3, hp))):
            value = self.get_parameter(name).value
            if value >= 0:
                setter(state, value)
        self.set_base_hp(state, state.event_data.base_hp)
        self.set_teammate_hp(state, 7, state.robot_status.current_hp)
        if not self.get_parameter("gazebo_navigation").value:
            state.ground_positions.standard_3_position = make_point(
                self.get_parameter("teammate_x").value,
                self.get_parameter("teammate_y").value)

    def resolve_scenario(self) -> str:
        if self.scenario == "match":
            return "match"
        if self.scenario != "auto":
            return self.scenario
        elapsed = (self.get_clock().now() - self.scenario_start_time).nanoseconds / 1e9
        index = int(elapsed / max(self.scenario_period_sec, 0.1)) % len(SCENARIOS)
        return SCENARIOS[index]

    def base_state(self, name: str) -> ScenarioState:
        game_status = GameStatus()
        game_status.game_progress = game_status.RUNNING
        game_status.stage_remain_time = 360

        robot_status = RobotStatus()
        robot_status.robot_id = 7 if self.team_color == "red" else 107
        robot_status.robot_level = 1
        robot_status.current_hp = 400
        robot_status.maximum_hp = 400
        robot_status.shooter_barrel_cooling_value = 30
        robot_status.shooter_barrel_heat_limit = 400
        robot_status.shooter_17mm_1_barrel_heat = 80
        robot_status.projectile_allowance_17mm = 120
        robot_status.remaining_gold_coin = 0
        robot_status.robot_pos.position = make_point(1.0, -4.0)
        robot_status.robot_pos.orientation.w = 1.0

        event_data = EventData()
        event_data.center_gain_zone = event_data.OCCUPIED_FRIEND
        event_data.base_hp = 6000
        event_data.fortress_gain_zone = event_data.OCCUPIED_FRIEND
        event_data.supply_zone = event_data.OCCUPIED_FRIEND
        event_data.non_overlapping_supply_zone = event_data.OCCUPIED_FRIEND
        event_data.overlapping_supply_zone = event_data.OCCUPIED_FRIEND

        rfid_status = RfidStatus()

        hp = GameRobotHP()
        hp.red_1_robot_hp = 400
        hp.red_2_robot_hp = 400
        hp.red_3_robot_hp = 400
        hp.red_4_robot_hp = 400
        hp.red_7_robot_hp = 400
        hp.red_outpost_hp = 0
        hp.red_base_hp = 6000
        hp.blue_1_robot_hp = 400
        hp.blue_2_robot_hp = 400
        hp.blue_3_robot_hp = 400
        hp.blue_4_robot_hp = 400
        hp.blue_7_robot_hp = 400
        hp.blue_outpost_hp = 0
        hp.blue_base_hp = 6000

        positions = GroundRobotPosition()
        positions.hero_position = make_point(2.0, -1.0)
        positions.engineer_position = make_point(3.0, -2.0)
        positions.standard_3_position = make_point(4.0, 0.4)
        positions.standard_4_position = make_point(5.0, -1.2)

        target = Target()
        target.tracking = False
        target.id = ""
        target.armors_num = 0
        target.position = make_point(7.0, -2.0)
        target.yaw = math.pi

        return ScenarioState(
            name,
            game_status,
            robot_status,
            event_data,
            rfid_status,
            hp,
            positions,
            target,
            name,
        )

    def build_scenario(self, scenario_name: str) -> ScenarioState:
        if scenario_name == "match":
            return self.build_match_state()
        builders: Dict[str, Callable[[], ScenarioState]] = {
            "hold_safe": self.scenario_hold_safe,
            "resupply_home": self.scenario_resupply_home,
            "defend_base": self.scenario_defend_base,
            "defend_fortress": self.scenario_defend_fortress,
            "cover_teammate": self.scenario_cover_teammate,
            "attack_outpost": self.scenario_attack_outpost,
            "capture_point": self.scenario_capture_point,
            "gather_info": self.scenario_gather_info,
            "resupply_complete": self.scenario_resupply_complete,
            "outpost_destroyed": self.scenario_gather_info,
            "game_over": self.scenario_game_over,
        }
        return builders[scenario_name]()

    def build_match_state(self) -> ScenarioState:
        elapsed = (self.get_clock().now() - self.scenario_start_time).nanoseconds / 1e9
        match_elapsed = elapsed * self.speed
        preparation_sec = 5.0
        countdown_sec = 5.0

        if match_elapsed < preparation_sec:
            state = self.base_state("match")
            state.game_status.game_progress = state.game_status.PREPARATION
            state.game_status.stage_remain_time = max(
                0, math.ceil(preparation_sec - match_elapsed)
            )
            state.event_name = "preparation"
            return state

        if match_elapsed < preparation_sec + countdown_sec:
            state = self.base_state("match")
            state.game_status.game_progress = state.game_status.COUNT_DOWN
            state.game_status.stage_remain_time = max(
                0, math.ceil(preparation_sec + countdown_sec - match_elapsed)
            )
            state.event_name = "countdown"
            return state

        running_elapsed = match_elapsed - preparation_sec - countdown_sec
        if running_elapsed >= self.match_duration_sec:
            state = self.base_state("match")
            state.game_status.game_progress = state.game_status.GAME_OVER
            state.game_status.stage_remain_time = 0
            state.event_name = "match_complete"
            return state

        if self.match_state is None:
            self.match_state = self.base_state("match")
            self.last_simulated_time = 0.0
        state = self.match_state
        dt = max(0.0, running_elapsed - self.last_simulated_time)
        self.last_simulated_time = running_elapsed
        self.advance_match_state(state, running_elapsed, dt)
        state.game_status.game_progress = state.game_status.RUNNING
        state.game_status.stage_remain_time = max(
            0, math.ceil(self.match_duration_sec - running_elapsed)
        )
        return state

    def advance_match_state(self, state: ScenarioState, elapsed: float, dt: float) -> None:
        """Advance a deterministic continuous world model by one simulation step."""
        # Smoothly move friendly units and the observed target.
        state.robot_status.robot_pos.position.x = 1.0 + 0.012 * elapsed
        state.robot_status.robot_pos.position.y = -4.0 + 0.45 * math.sin(elapsed / 12.0)
        state.ground_positions.standard_3_position.x = 4.0 + 0.8 * math.sin(elapsed / 9.0)
        state.ground_positions.standard_3_position.y = 1.0 + 0.6 * math.cos(elapsed / 11.0)
        state.target.position.x = 7.0 + 1.2 * math.sin(elapsed / 13.0)
        state.target.position.y = -2.5 + 0.8 * math.cos(elapsed / 10.0)
        state.target.yaw = math.atan2(
            state.robot_status.robot_pos.position.y - state.target.position.y,
            state.robot_status.robot_pos.position.x - state.target.position.x,
        )

        # Deterministic pressure windows create state-driven transitions.
        resource_pressure = 82.0 <= elapsed < 105.0
        defense_pressure = 125.0 <= elapsed < 150.0
        teammate_pressure = 58.0 <= elapsed < 82.0
        center_pressure = (18.0 <= elapsed < 42.0) or elapsed >= 158.0
        target_pressure = (38.0 <= elapsed < 58.0) or (150.0 <= elapsed < 178.0)

        if resource_pressure:
            state.robot_status.current_hp = max(90, int(150 - (elapsed - 82.0) * 1.4))
            state.robot_status.projectile_allowance_17mm = max(10, int(42 - (elapsed - 82.0)))
            state.robot_status.shooter_17mm_1_barrel_heat = min(390, int(320 + (elapsed - 82.0) * 2.5))
        else:
            state.robot_status.current_hp = min(400, int(state.robot_status.current_hp + dt * 2.0))
            state.robot_status.projectile_allowance_17mm = min(120, int(state.robot_status.projectile_allowance_17mm + dt * 0.8))
            state.robot_status.shooter_17mm_1_barrel_heat = max(80, int(state.robot_status.shooter_17mm_1_barrel_heat - dt * 8.0))

        if teammate_pressure:
            self.set_teammate_hp(state, 3, max(80, int(210 - (elapsed - 58.0) * 2.0)))
        else:
            self.set_teammate_hp(state, 3, min(400, int(self.teammate_hp_value(state, 3) + dt * 2.5)))

        state.event_data.center_gain_zone = (
            state.event_data.OCCUPIED_ENEMY if center_pressure else state.event_data.OCCUPIED_FRIEND
        )
        state.event_data.base_hp = max(1800, int(6000 - max(0.0, elapsed - 120.0) * 55.0)) if defense_pressure else min(6000, state.event_data.base_hp + int(dt * 20.0))
        self.set_base_hp(state, state.event_data.base_hp)
        state.event_data.fortress_gain_zone = state.event_data.OCCUPIED_ENEMY if defense_pressure else state.event_data.OCCUPIED_FRIEND
        state.target.tracking = target_pressure
        state.target.id = "outpost" if target_pressure else ""
        state.target.armors_num = 1 if target_pressure else 0
        self.set_outpost_hp(state, max(200, int(1800 - max(0.0, elapsed - 38.0) * 12.0)) if target_pressure else 0)
        if resource_pressure:
            state.event_name = "resource_pressure"
        elif defense_pressure:
            state.event_name = "home_defense_pressure"
        elif teammate_pressure:
            state.event_name = "teammate_under_fire"
        elif target_pressure:
            state.event_name = "target_detected"
        elif center_pressure:
            state.event_name = "center_contest"
        else:
            state.event_name = "recon_patrol"

    def teammate_hp_value(self, state: ScenarioState, robot_id: int) -> int:
        prefix = "red" if self.team_color == "red" else "blue"
        return int(getattr(state.all_robot_hp, f"{prefix}_{robot_id}_robot_hp"))

    def set_teammate_hp(self, state: ScenarioState, robot_id: int, hp_value: int) -> None:
        prefix = "red" if self.team_color == "red" else "blue"
        setattr(state.all_robot_hp, f"{prefix}_{robot_id}_robot_hp", hp_value)

    def set_outpost_hp(self, state: ScenarioState, hp_value: int) -> None:
        prefix = "blue" if self.team_color == "red" else "red"
        setattr(state.all_robot_hp, f"{prefix}_outpost_hp", hp_value)

    def set_base_hp(self, state: ScenarioState, hp_value: int) -> None:
        prefix = "red" if self.team_color == "red" else "blue"
        setattr(state.all_robot_hp, f"{prefix}_base_hp", hp_value)

    def simulation_state_text(self, state: ScenarioState) -> str:
        elapsed = (self.get_clock().now() - self.scenario_start_time).nanoseconds / 1e9
        simulated_elapsed = min(
            elapsed * self.speed,
            5.0 + 5.0 + self.match_duration_sec,
        )
        return (
            f"scenario={self.scenario} phase={self.game_progress_text(state.game_status)} "
            f"event={state.event_name or state.name} sim_time={simulated_elapsed:.1f} "
            f"remaining={state.game_status.stage_remain_time} world_state_id={self.world_state_id} "
            f"dt={self.simulation_step_sec:.1f} seed={self.random_seed}"
        )

    def log_state_if_needed(self, state: ScenarioState, simulation_state: str) -> None:
        now_sec = (self.get_clock().now() - self.scenario_start_time).nanoseconds / 1e9
        signature = (
            self.game_progress_text(state.game_status),
            state.event_name,
            state.robot_status.current_hp,
            state.event_data.base_hp,
        )
        changed = signature != self.last_output_signature
        periodic = now_sec - self.last_output_time >= self.simulation_log_period_sec
        if changed or periodic:
            self.current_scenario_name = state.event_name
            self.last_output_signature = signature
            self.last_output_time = now_sec
            self.get_logger().info(
                f"[RefereeSim] {simulation_state} "
                f"robot_hp={state.robot_status.current_hp} "
                f"ammo={state.robot_status.projectile_allowance_17mm} "
                f"heat={state.robot_status.shooter_17mm_1_barrel_heat} "
                f"base_hp={state.event_data.base_hp} "
                f"outpost_hp={self.outpost_hp(state)}"
            )

    def game_progress_text(self, game_status: GameStatus) -> str:
        mapping = {
            game_status.NOT_START: "NOT_START",
            game_status.PREPARATION: "PREPARATION",
            game_status.SELF_CHECKING: "SELF_CHECKING",
            game_status.COUNT_DOWN: "COUNT_DOWN",
            game_status.RUNNING: "RUNNING",
            game_status.GAME_OVER: "GAME_OVER",
        }
        return mapping.get(game_status.game_progress, str(game_status.game_progress))

    def outpost_hp(self, state: ScenarioState) -> int:
        return (
            state.all_robot_hp.blue_outpost_hp
            if self.team_color == "red"
            else state.all_robot_hp.red_outpost_hp
        )

    def scenario_hold_safe(self) -> ScenarioState:
        state = self.base_state("hold_safe")
        state.game_status.game_progress = state.game_status.NOT_START
        state.game_status.stage_remain_time = 420
        return state

    def scenario_resupply_home(self) -> ScenarioState:
        state = self.base_state("resupply_home")
        state.robot_status.current_hp = 120
        state.robot_status.projectile_allowance_17mm = 20
        state.robot_status.shooter_17mm_1_barrel_heat = 360
        return state

    def scenario_defend_base(self) -> ScenarioState:
        state = self.base_state("defend_base")
        state.event_data.base_hp = 3200
        if self.team_color == "red":
            state.all_robot_hp.red_base_hp = 3200
        else:
            state.all_robot_hp.blue_base_hp = 3200
        return state

    def scenario_defend_fortress(self) -> ScenarioState:
        state = self.base_state("defend_fortress")
        state.event_data.fortress_gain_zone = state.event_data.OCCUPIED_ENEMY
        return state

    def scenario_cover_teammate(self) -> ScenarioState:
        state = self.base_state("cover_teammate")
        state.ground_positions.standard_3_position = make_point(4.0, 1.8)
        if self.team_color == "red":
            state.all_robot_hp.red_3_robot_hp = 20
        else:
            state.all_robot_hp.blue_3_robot_hp = 20
        return state

    def scenario_attack_outpost(self) -> ScenarioState:
        state = self.base_state("attack_outpost")
        state.target.tracking = True
        state.target.id = "outpost"
        state.target.armors_num = 1
        state.target.position = make_point(7.2, -2.8)
        self.set_outpost_hp(state, 1500)
        return state

    def scenario_capture_point(self) -> ScenarioState:
        state = self.base_state("capture_point")
        state.game_status.stage_remain_time = 20
        state.event_data.center_gain_zone = state.event_data.OCCUPIED_ENEMY
        return state

    def scenario_resupply_complete(self) -> ScenarioState:
        state = self.base_state("resupply_complete")
        state.robot_status.projectile_allowance_17mm = 150
        return state

    def scenario_game_over(self) -> ScenarioState:
        state = self.base_state("game_over")
        state.game_status.game_progress = state.game_status.GAME_OVER
        state.game_status.stage_remain_time = 0
        return state

    def scenario_gather_info(self) -> ScenarioState:
        return self.base_state("gather_info")


def main(args: List[str] = None) -> None:
    rclpy.init(args=args)
    node = SentryRefereeSimulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
