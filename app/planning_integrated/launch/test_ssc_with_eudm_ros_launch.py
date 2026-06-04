#!/usr/bin/env python3
"""
@file test_ssc_with_eudm_ros_launch.py
@brief EUDM行为规划器 + SSC运动规划器 联合启动脚本 (ROS2 Launch文件)

[架构定位]
本 launch 文件是 EPSILON 自动驾驶规划系统的入口启动脚本，
负责启动 test_ssc_with_eudm 节点（EUDM行为规划器 + SSC运动规划器的集成入口）。

[功能说明]
1. 声明启动参数 (arena_info_static_topic, arena_info_dynamic_topic, ctrl_topic, playground)
2. 通过 PathJoinSubstitution 动态拼接各模块配置文件路径
3. 配置 remappings 将内部话题名映射到外部话题名

[参数说明]
- arena_info_static_topic: 静态竞技场信息话题名 (车道拓扑、地图结构等不变信息)
- arena_info_dynamic_topic: 动态竞技场信息话题名 (车辆实时位置、速度等动态信息)
- ctrl_topic: 发布自车控制指令的话题名 (vehicle_msgs::msg::ControlSignal)
- playground: 运行场景名称 (如 highway_v1.0), 决定加载哪个场景的地图和车辆配置

[话题重映射]
内部话题 -> 外部话题 (由启动参数配置):
- 'arena_info_static' -> arena_info_static_topic (默认 /arena_info_static)
- 'arena_info_dynamic' -> arena_info_dynamic_topic (默认 /arena_info_dynamic)
- 'ctrl' -> ctrl_topic (默认 /ctrl/agent_0)

[与 test_ssc_with_mpdm_ros_launch.py 的关系]
两个 launch 文件启动不同的行为规划器后端:
- 本文件: EUDM (Efficient Uncertainty-aware Decision Making) —— 适合结构化高速场景
- test_ssc_with_mpdm_ros_launch.py: MPDM (Multi-Policy Decision Making) —— 适合复杂交通场景
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    """
    generate_launch_description —— ROS2 Launch 系统的入口函数

    返回 LaunchDescription 对象, 其中包含:
    1. DeclareLaunchArgument: 声明可配置的启动参数, 允许用户通过命令行覆盖默认值
    2. Node: 定义要启动的 ROS2 节点及其参数和话题映射
    3. LogInfo: 启动时打印关键参数值, 便于调试和确认配置

    [数据流: 配置文件路径拼接]
    get_package_share_directory('playgrounds')           # 获取 playgrounds 包的 share 目录
    + LaunchConfiguration('playground')                  # 场景名称, 如 'highway_v1.0'
    + 'agent_config.json'                                # 智能体配置文件名
    = 完整路径: .../share/playgrounds/highway_v1.0/agent_config.json
    """

    # ---------- 声明启动参数 (DeclareLaunchArgument) ----------
    # 静态竞技场信息话题: 包含地图、车道拓扑等不随仿真时间变化的信息
    arena_info_static_topic = DeclareLaunchArgument(
        'arena_info_static_topic', default_value='/arena_info_static'
    )
    # 动态竞技场信息话题: 包含所有车辆实时位置、速度、朝向等动态状态
    arena_info_dynamic_topic = DeclareLaunchArgument(
        'arena_info_dynamic_topic', default_value='/arena_info_dynamic'
    )
    # 控制指令话题: SSC 运动规划器输出的方向盘转角/油门/刹车等底层控制量
    ctrl_topic = DeclareLaunchArgument(
        'ctrl_topic', default_value='/ctrl/agent_0'
    )
    # 场景名称参数: 决定加载哪个 playground 中的地图和车辆配置
    playground = DeclareLaunchArgument(
        'playground', default_value='highway_v1.0'
    )

    # ---------- 定义要启动的节点 ----------
    test_ssc_with_eudm_node = Node(
        package='planning_integrated',         # 所属 ROS2 功能包
        executable='test_ssc_with_eudm',       # 可执行文件名 (对应 CMakeLists.txt 中的 add_executable)
        name='test_ssc_with_eudm_0',           # 节点实例名称 (可在 ROS2 网络中唯一标识)
        output='screen',                       # 将标准输出打印到终端, 便于调试
        parameters=[{                          # 传入节点的 ROS2 参数字典
            'ego_id': 0,                       # 自车ID: 默认为 0, 对应物理仿真器中的 agent_0
            'desired_vel': 20.0,               # 期望速度 (m/s): 自车的目标巡航速度
            'use_sim_state': True,             # 使用仿真器状态: True 表示从仿真器获取状态而非真实传感器
            # 智能体配置文件路径: 定义车辆物理参数、传感器 FOV 等
            'agent_config_path': PathJoinSubstitution([
                get_package_share_directory('playgrounds'),
                LaunchConfiguration('playground'),
                'agent_config.json'
            ]),
            # EUDM 行为规划器配置文件路径: protobuf 格式, 定义 EUDM 的决策参数
            'bp_config_path': PathJoinSubstitution([
                get_package_share_directory('eudm_planner'),
                'config',
                'eudm_config.pb.txt'
            ]),
            # SSC 运动规划器配置文件路径: protobuf 格式, 定义轨迹优化参数
            'ssc_config_path': PathJoinSubstitution([
                get_package_share_directory('ssc_planner'),
                'config',
                'ssc_config.pb.txt'
            ])
        }],
        # remappings: 将节点内部使用的话题名重新映射到外部话题名
        remappings=[
            ('arena_info_static', LaunchConfiguration('arena_info_static_topic')),
            ('arena_info_dynamic', LaunchConfiguration('arena_info_dynamic_topic')),
            ('ctrl', LaunchConfiguration('ctrl_topic'))
        ]
    )

    # ---------- 组装 LaunchDescription ----------
    return LaunchDescription([
        arena_info_static_topic,
        arena_info_dynamic_topic,
        ctrl_topic,
        playground,
        # 启动时打印关键参数, 便于确认配置是否正确的日志信息
        LogInfo(msg=['arena_info_static_topic: ', LaunchConfiguration('arena_info_static_topic')]),
        LogInfo(msg=['arena_info_dynamic_topic: ', LaunchConfiguration('arena_info_dynamic_topic')]),
        LogInfo(msg=['ctrl_topic: ', LaunchConfiguration('ctrl_topic')]),
        LogInfo(msg=['playground: ', LaunchConfiguration('playground')]),
        LogInfo(msg="Launching node..."),
        test_ssc_with_eudm_node
    ])
