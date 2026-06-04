#!/usr/bin/env python3
"""
@file onlane_ego_agent_launch.py
@brief 自车 (Ego Vehicle) AI 智能体启动脚本 (ROS2 Launch文件)

[架构定位]
本 launch 文件负责启动单个 onlane_ai_agent 节点实例作为自车 (agent_0),
即使用 AI 规划器 (MPDM + IDM前向仿真) 来控制自车, 而非使用 planning_integrated
模块中的完整规划流水线 (EUDM/MPDM + SSC)。

[使用场景]
1. 纯 AI 驾驶模拟: 所有车辆 (含自车) 都由 onlane_ai_agent 控制,
   适用于快速验证交通流模型、评估行为策略等不需要高精度运动规划的场合
2. 与 planning_integrated 对比: 当需要将 AI 控制与完整规划流水线对比时,
   可以先启动本 launch 文件测试 AI 驾驶基线, 再切换到 planning_integrated

[参数说明]
- arena_info_topic: 竞技场全量信息话题 (同时包含静态和动态)
- arena_info_static_topic: 静态竞技场信息话题 (车道拓扑、地图结构)
- arena_info_dynamic_topic: 动态竞技场信息话题 (车辆实时位置、速度)
- global_desired_vel: 自车的基准期望速度 (m/s), 默认 10.0
- global_autonomous_level: 自车的自主等级, 默认 3 (Level 3)
- playground: 运行场景名称 (默认 highway_v1.0)

[与 onlane_ai_agent_launch.py 的区别]
- 本文件: 只启动 agent_0 (自车), ctrl 映射到 /ctrl/agent_0
- 本文件没有 aggressiveness_level 参数 (自车通常不需要特别激进)
- onlane_ai_agent_launch.py: 批量启动 agent_1..agent_10 (10个周围车辆)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    """
    generate_launch_description —— ROS2 Launch 系统入口函数

    返回 LaunchDescription 对象, 其中包含:
    1. DeclareLaunchArgument: 声明 6 个可配置的启动参数
    2. LogInfo: 启动时打印所有参数值
    3. 单个 onlane_ai_agent Node 实例 (agent_0, 作为自车)
    """

    # ---------- 声明话题名相关启动参数 ----------
    arena_info_topic = DeclareLaunchArgument(
        'arena_info_topic', default_value='/arena_info'
    )
    arena_info_static_topic = DeclareLaunchArgument(
        'arena_info_static_topic', default_value='/arena_info_static'
    )
    arena_info_dynamic_topic = DeclareLaunchArgument(
        'arena_info_dynamic_topic', default_value='/arena_info_dynamic'
    )
    # 期望速度: 自车的基准巡航速度 (m/s)
    global_desired_vel = DeclareLaunchArgument(
        'global_desired_vel', default_value='10.0'
    )
    # 自主等级: Level 3 = 有条件自动驾驶
    global_autonomous_level = DeclareLaunchArgument(
        'global_autonomous_level', default_value='3'
    )
    # 场景名称, 决定加载哪个 playground 的配置文件
    playground = DeclareLaunchArgument(
        'playground', default_value='highway_v1.0'
    )

    # ---------- 定义自车节点 (agent_0) ----------
    onlane_ai_agent_node_0 = Node(
        package='ai_agent_planner',              # 功能包名称
        executable='onlane_ai_agent',            # 可执行文件名
        name='onlane_ai_agent_0',                # 节点实例名称
        output='screen',                         # 将标准输出打印到终端
        parameters=[{                            # 自车参数
            'ego_id': 0,                         # 自车 ID 固定为 0
            # 智能体配置文件路径
            'agent_config_path': PathJoinSubstitution([
                get_package_share_directory('playgrounds'),
                LaunchConfiguration('playground'),
                'agent_config.json'
            ]),
            'desired_vel': LaunchConfiguration('global_desired_vel'),
            'autonomous_level': LaunchConfiguration('global_autonomous_level')
            # 注意: 不设置 aggressiveness_level, 使用默认值 (通常是中等偏保守)
        }],
        remappings=[
            ('arena_info', LaunchConfiguration('arena_info_topic')),
            ('arena_info_static', LaunchConfiguration('arena_info_static_topic')),
            ('arena_info_dynamic', LaunchConfiguration('arena_info_dynamic_topic')),
            # 自车控制指令映射到 /ctrl/agent_0
            ('ctrl', '/ctrl/agent_0')
        ]
    )

    # ---------- 组装 LaunchDescription ----------
    return LaunchDescription([
        arena_info_topic,
        arena_info_static_topic,
        arena_info_dynamic_topic,
        global_desired_vel,
        global_autonomous_level,
        playground,
        LogInfo(msg=['arena_info_topic: ', LaunchConfiguration('arena_info_topic')]),
        LogInfo(msg=['arena_info_static_topic: ', LaunchConfiguration('arena_info_static_topic')]),
        LogInfo(msg=['arena_info_dynamic_topic: ', LaunchConfiguration('arena_info_dynamic_topic')]),
        LogInfo(msg=['global_desired_vel: ', LaunchConfiguration('global_desired_vel')]),
        LogInfo(msg=['global_autonomous_level: ', LaunchConfiguration('global_autonomous_level')]),
        LogInfo(msg=['playground: ', LaunchConfiguration('playground')]),
        LogInfo(msg="Launching node..."),
        onlane_ai_agent_node_0
    ])
