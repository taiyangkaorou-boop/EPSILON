#!/usr/bin/env python3
"""
@file onlane_ai_agent_launch.py
@brief 周围交通车辆 AI 智能体批量启动脚本 (ROS2 Launch文件)

[架构定位]
本 launch 文件负责批量启动 10 个 onlane_ai_agent 节点实例 (agent_1 到 agent_10),
每个节点作为一个独立的周围交通车辆 (social vehicle), 运行自己的 MPDM 行为规划器
和 IDM 前向仿真器, 与自车 (agent_0) 在同一仿真环境中交互。

[功能说明]
1. 声明全局启动参数 (arena_info_topics, desired_vel, autonomous_level, aggressiveness_level, playground)
2. 通过 for 循环批量创建 10 个 Node 实例 (agent_1 .. agent_10)
3. 每个智能体的 ctrl_topic 自动映射为 /ctrl/agent_{i}

[参数说明]
- arena_info_topic: 竞技场全量信息话题 (同时包含静态和动态信息)
- arena_info_static_topic: 静态竞技场信息话题 (车道拓扑、地图结构)
- arena_info_dynamic_topic: 动态竞技场信息话题 (车辆实时位置、速度)
- global_desired_vel: 所有 AI 智能体的基准期望速度 (m/s), 默认 10.0
- global_autonomous_level: 所有 AI 智能体的自主等级 (0-5), 默认 3
- global_aggressiveness_level: 所有 AI 智能体的激进程度 (1-5), 默认 4
- playground: 运行场景名称 (默认 highway_lite)

[与 onlane_ego_agent_launch.py 的区别]
- 本文件: 启动 agent_1 到 agent_10 (周围车辆), ctrl 映射到 /ctrl/agent_{i}
- onlane_ego_agent_launch.py: 只启动 agent_0 (自车), ctrl 映射到 /ctrl/agent_0
- 本文件额外声明 global_aggressiveness_level (周围车辆通常更激进)
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
    1. DeclareLaunchArgument: 声明 7 个可配置的启动参数
    2. LogInfo: 启动时打印所有参数值
    3. 通过 for 循环批量创建的 10 个 onlane_ai_agent Node 实例
    """

    # ---------- 声明话题名相关启动参数 ----------
    # 竞技场全量信息话题 (同时包含静态+动态)
    arena_info_topic = DeclareLaunchArgument(
        'arena_info_topic', default_value='/arena_info'
    )
    # 静态竞技场信息话题
    arena_info_static_topic = DeclareLaunchArgument(
        'arena_info_static_topic', default_value='/arena_info_static'
    )
    # 动态竞技场信息话题
    arena_info_dynamic_topic = DeclareLaunchArgument(
        'arena_info_dynamic_topic', default_value='/arena_info_dynamic'
    )
    # 全局期望速度: 所有周围车辆共享的基准巡航速度
    global_desired_vel = DeclareLaunchArgument(
        'global_desired_vel', default_value='10.0'
    )
    # 全局自主等级: Level 3 = 有条件自动驾驶
    global_autonomous_level = DeclareLaunchArgument(
        'global_autonomous_level', default_value='3'
    )
    # 全局激进程度: 4 表示较为激进的驾驶风格 (跟车距离更近, 换道更果断)
    global_aggressiveness_level = DeclareLaunchArgument(
        'global_aggressiveness_level', default_value='4'
    )
    # 场景名称, 决定加载哪个 playground 的配置文件
    playground = DeclareLaunchArgument(
        'playground', default_value='highway_lite'
    )

    # ---------- 批量创建 10 个 AI 智能体节点 (agent_1 .. agent_10) ----------
    onlane_ai_agent_nodes = []
    for i in range(1, 11):
        onlane_ai_agent_nodes.append(Node(
            package='ai_agent_planner',              # 功能包名称
            executable='onlane_ai_agent',            # 可执行文件名
            name=f'onlane_ai_agent_{i}',             # 节点实例名称: onlane_ai_agent_1 .. onlane_ai_agent_10
            output='screen',                         # 将标准输出打印到终端
            parameters=[{                            # 每个智能体的参数字典
                'ego_id': i,                         # 智能体 ID: 1-10, 对应物理仿真器中的 agent_1 到 agent_10
                # 智能体配置文件路径: 所有智能体共享同一个 agent_config.json
                'agent_config_path': PathJoinSubstitution([
                    get_package_share_directory('playgrounds'),
                    LaunchConfiguration('playground'),
                    'agent_config.json'
                ]),
                'desired_vel': LaunchConfiguration('global_desired_vel'),
                'autonomous_level': LaunchConfiguration('global_autonomous_level'),
                'aggressiveness_level': LaunchConfiguration('global_aggressiveness_level')
            }],
            remappings=[
                # 话题重映射: 内部话题 -> 外部话题
                ('arena_info', LaunchConfiguration('arena_info_topic')),
                ('arena_info_static', LaunchConfiguration('arena_info_static_topic')),
                ('arena_info_dynamic', LaunchConfiguration('arena_info_dynamic_topic')),
                # 每个智能体发布到独立的 ctrl 话题, 避免冲突
                ('ctrl', f'/ctrl/agent_{i}')
            ]
        ))

    # ---------- 组装 LaunchDescription ----------
    return LaunchDescription([
        arena_info_topic,
        arena_info_static_topic,
        arena_info_dynamic_topic,
        global_desired_vel,
        global_autonomous_level,
        global_aggressiveness_level,
        playground,
        # 启动日志: 打印所有参数值便于确认配置
        LogInfo(msg=['arena_info_topic: ', LaunchConfiguration('arena_info_topic')]),
        LogInfo(msg=['arena_info_static_topic: ', LaunchConfiguration('arena_info_static_topic')]),
        LogInfo(msg=['arena_info_dynamic_topic: ', LaunchConfiguration('arena_info_dynamic_topic')]),
        LogInfo(msg=['global_desired_vel: ', LaunchConfiguration('global_desired_vel')]),
        LogInfo(msg=['global_autonomous_level: ', LaunchConfiguration('global_autonomous_level')]),
        LogInfo(msg=['global_aggressiveness_level: ', LaunchConfiguration('global_aggressiveness_level')]),
        LogInfo(msg=['playground: ', LaunchConfiguration('playground')]),
        LogInfo(msg="Launching nodes..."),
        # Python 展开操作符: 将列表中的所有 Node 展开为 LaunchDescription 的独立项
        *onlane_ai_agent_nodes
    ])
