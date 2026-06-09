#!/usr/bin/env python3
"""
@file risk_experiment_closed_loop_launch.py
@brief 风险感知 SSC 论文实验闭环启动脚本。

该 launch 用于 MVP-13 实验闭环：同时启动物理仿真器和 planning_integrated，
让 phy_simulator 发布 arena_info，planning_integrated 订阅场景并发布 ctrl，
从而形成可由 risk_experiment_batch.py 批量调用的完整闭环实验入口。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def planning_launch_source(backend: str) -> PythonLaunchDescriptionSource:
    """返回指定后端对应的 planning_integrated launch 文件。"""
    return PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            get_package_share_directory("planning_integrated"),
            "launch",
            f"test_ssc_with_{backend}_ros_launch.py",
        ])
    ])


def backend_is(backend: str) -> IfCondition:
    """生成 planner_backend 是否等于指定后端的 launch 条件。"""
    return IfCondition(PythonExpression([
        "'",
        LaunchConfiguration("planner_backend"),
        f"' == '{backend}'",
    ]))


def generate_launch_description():
    """生成闭环实验 LaunchDescription。"""
    # 统一话题名，保证仿真器和规划器在同一个 arena/ctrl 闭环里通信。
    arena_info_static_topic = DeclareLaunchArgument(
        "arena_info_static_topic", default_value="/arena_info_static"
    )
    arena_info_dynamic_topic = DeclareLaunchArgument(
        "arena_info_dynamic_topic", default_value="/arena_info_dynamic"
    )
    ctrl_topic = DeclareLaunchArgument(
        "ctrl_topic", default_value="/ctrl/agent_0"
    )
    # planner_backend 只允许 eudm 或 mpdm；文件名拼接与现有 launch 命名保持一致。
    planner_backend = DeclareLaunchArgument(
        "planner_backend", default_value="eudm", choices=["eudm", "mpdm"]
    )
    playground = DeclareLaunchArgument(
        "playground", default_value="highway_v1.0"
    )
    ssc_config_path = DeclareLaunchArgument(
        "ssc_config_path",
        default_value=PathJoinSubstitution([
            get_package_share_directory("ssc_planner"),
            "config",
            "ssc_config.pb.txt",
        ]),
    )
    # MVP-16: 默认不启动脚本化周车；批量实验或指定场景可显式打开。
    enable_scripted_risk_actors = DeclareLaunchArgument(
        "enable_scripted_risk_actors", default_value="false"
    )
    risk_actor_script_path = DeclareLaunchArgument(
        "risk_actor_script_path",
        default_value=PathJoinSubstitution([
            get_package_share_directory("playgrounds"),
            LaunchConfiguration("playground"),
            "risk_actor_script.json",
        ]),
    )
    risk_actor_publish_rate_hz = DeclareLaunchArgument(
        "risk_actor_publish_rate_hz", default_value="50.0"
    )

    # 批量实验不依赖物理手柄，直接启动仿真节点，避免 joy_node 在无手柄机器上失败。
    vehicle_info_path = PathJoinSubstitution([
        get_package_share_directory("playgrounds"),
        LaunchConfiguration("playground"),
        "vehicle_set.json",
    ])
    map_path = PathJoinSubstitution([
        get_package_share_directory("playgrounds"),
        LaunchConfiguration("playground"),
        "obstacles_norm.json",
    ])
    lane_net_path = PathJoinSubstitution([
        get_package_share_directory("playgrounds"),
        LaunchConfiguration("playground"),
        "lane_net_norm.json",
    ])
    simulator_node = Node(
        package="phy_simulator",
        executable="phy_simulator_planning_node",
        name="phy_simulator_planning_node",
        output="screen",
        parameters=[{
            "vehicle_info_path": vehicle_info_path,
            "map_path": map_path,
            "lane_net_path": lane_net_path,
        }],
        remappings=[
            ("arena_info_static", LaunchConfiguration("arena_info_static_topic")),
            ("arena_info_dynamic", LaunchConfiguration("arena_info_dynamic_topic")),
        ],
    )
    scripted_risk_actor_node = Node(
        package="planning_integrated",
        executable="scripted_risk_actor_node.py",
        name="scripted_risk_actor_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_scripted_risk_actors")),
        parameters=[{
            "script_path": LaunchConfiguration("risk_actor_script_path"),
            "vehicle_info_path": vehicle_info_path,
            "publish_rate_hz": LaunchConfiguration("risk_actor_publish_rate_hz"),
        }],
    )

    planning_launch_arguments = {
        "arena_info_static_topic": LaunchConfiguration("arena_info_static_topic"),
        "arena_info_dynamic_topic": LaunchConfiguration("arena_info_dynamic_topic"),
        "ctrl_topic": LaunchConfiguration("ctrl_topic"),
        "playground": LaunchConfiguration("playground"),
        "ssc_config_path": LaunchConfiguration("ssc_config_path"),
    }
    eudm_planning_launch = IncludeLaunchDescription(
        planning_launch_source("eudm"),
        condition=backend_is("eudm"),
        launch_arguments=planning_launch_arguments.items(),
    )
    mpdm_planning_launch = IncludeLaunchDescription(
        planning_launch_source("mpdm"),
        condition=backend_is("mpdm"),
        launch_arguments=planning_launch_arguments.items(),
    )

    return LaunchDescription([
        arena_info_static_topic,
        arena_info_dynamic_topic,
        ctrl_topic,
        planner_backend,
        playground,
        ssc_config_path,
        enable_scripted_risk_actors,
        risk_actor_script_path,
        risk_actor_publish_rate_hz,
        LogInfo(msg=["planner_backend: ", LaunchConfiguration("planner_backend")]),
        LogInfo(msg=["playground: ", LaunchConfiguration("playground")]),
        LogInfo(msg=["ssc_config_path: ", LaunchConfiguration("ssc_config_path")]),
        LogInfo(msg=["enable_scripted_risk_actors: ", LaunchConfiguration("enable_scripted_risk_actors")]),
        LogInfo(msg=["risk_actor_script_path: ", LaunchConfiguration("risk_actor_script_path")]),
        LogInfo(msg=["vehicle_info_path: ", vehicle_info_path]),
        LogInfo(msg=["map_path: ", map_path]),
        LogInfo(msg=["lane_net_path: ", lane_net_path]),
        LogInfo(msg="Launching closed-loop risk experiment..."),
        simulator_node,
        scripted_risk_actor_node,
        eudm_planning_launch,
        mpdm_planning_launch,
    ])
