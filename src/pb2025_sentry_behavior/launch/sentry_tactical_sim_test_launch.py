import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_sentry_behavior")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    scenario = LaunchConfiguration("scenario")
    scenario_period_sec = LaunchConfiguration("scenario_period_sec")
    team_color = LaunchConfiguration("team_color")
    publish_rate = LaunchConfiguration("publish_rate")
    match_duration_sec = LaunchConfiguration("match_duration_sec")
    speed = LaunchConfiguration("speed")
    random_seed = LaunchConfiguration("random_seed")
    simulation_log_period_sec = LaunchConfiguration("simulation_log_period_sec")
    start_simulator = LaunchConfiguration("start_simulator")
    start_scorer = LaunchConfiguration("start_scorer")
    start_observer = LaunchConfiguration("start_observer")
    log_file = LaunchConfiguration("log_file")
    with_behavior_tree = LaunchConfiguration("with_behavior_tree")
    log_level = LaunchConfiguration("log_level")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "team_color": team_color,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup_dir, "params", "sentry_behavior.yaml"),
                description="Parameter file used by the tactical scorer and behavior server",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("scenario", default_value="hold_safe"),
            DeclareLaunchArgument("scenario_period_sec", default_value="6.0"),
            DeclareLaunchArgument("team_color", default_value="blue"),
            DeclareLaunchArgument("publish_rate", default_value="5.0"),
            DeclareLaunchArgument("match_duration_sec", default_value="180.0"),
            DeclareLaunchArgument("speed", default_value="1.0"),
            DeclareLaunchArgument("random_seed", default_value="20260921"),
            DeclareLaunchArgument("simulation_log_period_sec", default_value="2.0"),
            DeclareLaunchArgument("start_simulator", default_value="true"),
            DeclareLaunchArgument("start_scorer", default_value="true"),
            DeclareLaunchArgument("start_observer", default_value="true"),
            DeclareLaunchArgument("log_file", default_value="auto"),
            DeclareLaunchArgument("with_behavior_tree", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            Node(
                package="pb2025_sentry_behavior",
                executable="sentry_referee_simulator.py",
                name="sentry_referee_simulator",
                output="screen",
                condition=IfCondition(start_simulator),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "scenario": scenario,
                        "scenario_period_sec": scenario_period_sec,
                        "team_color": team_color,
                        "publish_rate": ParameterValue(publish_rate, value_type=float),
                        "match_duration_sec": ParameterValue(
                            match_duration_sec, value_type=float
                        ),
                        "speed": ParameterValue(speed, value_type=float),
                        "random_seed": random_seed,
                        "simulation_log_period_sec": ParameterValue(
                            simulation_log_period_sec, value_type=float
                        ),
                    }
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="pb2025_sentry_behavior",
                executable="tactical_scorer_node",
                name="tactical_scorer_node",
                output="screen",
                condition=IfCondition(start_scorer),
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="pb2025_sentry_behavior",
                executable="sentry_tactical_visualizer.py",
                name="sentry_tactical_text_observer",
                output="screen",
                condition=IfCondition(start_observer),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "team_color": team_color,
                        "log_file": log_file,
                    }
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="pb2025_sentry_behavior",
                executable="pb2025_sentry_behavior_server",
                name="pb2025_sentry_behavior_server",
                output="screen",
                condition=IfCondition(with_behavior_tree),
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="pb2025_sentry_behavior",
                executable="pb2025_sentry_behavior_client",
                name="pb2025_sentry_behavior_client",
                output="screen",
                condition=IfCondition(with_behavior_tree),
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )
