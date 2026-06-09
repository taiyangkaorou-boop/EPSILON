#!/usr/bin/env python3
"""
@file test_ssc_with_mpdm_ros_launch.py
@brief MPDM行为规划器 + SSC运动规划器 联合启动脚本 (ROS2 Launch文件)

[架构定位]
本 launch 文件是 EPSILON 自动驾驶规划系统的入口启动脚本之一，
负责启动 test_ssc_with_mpdm 节点（MPDM行为规划器 + SSC运动规划器的集成入口）。

[功能说明]
1. 声明启动参数 (arena_info_static_topic, arena_info_dynamic_topic, ctrl_topic,
   playground, ssc_config_path)
2. 通过 PathJoinSubstitution 动态拼接各模块配置文件路径
3. 配置 remappings 将内部话题名映射到外部话题名

[参数说明]
- arena_info_static_topic: 静态竞技场信息话题名 (车道拓扑、地图结构等不变信息)
- arena_info_dynamic_topic: 动态竞技场信息话题名 (车辆实时位置、速度等动态信息)
- ctrl_topic: 发布自车控制指令的话题名 (vehicle_msgs::msg::ControlSignal)
- playground: 运行场景名称 (如 highway_lite), 决定加载哪个场景的地图和车辆配置
- ssc_config_path: SSC 配置文件路径，默认使用包内 baseline 配置；实验时可覆盖为
  `ssc_config_risk_corridor.pb.txt` 等风险消融配置

[话题重映射]
内部话题 -> 外部话题 (由启动参数配置):
- 'arena_info_static' -> arena_info_static_topic (默认 /arena_info_static)
- 'arena_info_dynamic' -> arena_info_dynamic_topic (默认 /arena_info_dynamic)
- 'ctrl' -> ctrl_topic (默认 /ctrl/agent_0)

[MPDM 特有注意事项]
- MPDM 不需要 bp_config_path 参数 (其策略内建在代码逻辑中)
- 默认 desired_vel 为 60.0 m/s (比 EUDM 的 20.0 m/s 更高, 适合高速验证场景)
- 默认 playground 为 highway_lite (比 highway_v1.0 更简化的高速场景)
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
    1. DeclareLaunchArgument: 声明可配置的启动参数
    2. LogInfo: 启动时打印关键参数值, 便于调试
    3. Node: 定义要启动的 test_ssc_with_mpdm 节点及其参数和话题映射

    [与 EUDM 版本的差异]
    - 不传入 bp_config_path (MPDM 不使用 protobuf 配置文件)
    - desired_vel 默认值为 60.0 m/s (EUDM 为 20.0 m/s)
    - playground 默认值为 highway_lite (EUDM 为 highway_v1.0)
    """
    # 默认 SSC 配置路径。MVP-11 允许命令行覆盖，方便做风险策略消融实验。
    default_ssc_config_path = PathJoinSubstitution([
        get_package_share_directory('ssc_planner'),
        'config',
        'ssc_config.pb.txt'
    ])

    return LaunchDescription([
        # ---------- 声明启动参数 ----------
        # 静态竞技场信息话题: 包含地图、车道拓扑等不随仿真时间变化的信息
        DeclareLaunchArgument(
            'arena_info_static_topic', default_value='/arena_info_static'
        ),
        # 动态竞技场信息话题: 包含所有车辆实时位置、速度、朝向等动态状态
        DeclareLaunchArgument(
            'arena_info_dynamic_topic', default_value='/arena_info_dynamic'
        ),
        # 控制指令话题: SSC 运动规划器输出的方向盘转角/油门/刹车等底层控制量
        DeclareLaunchArgument(
            'ctrl_topic', default_value='/ctrl/agent_0'
        ),
        # 场景名称参数: 决定加载哪个 playground 中的地图和车辆配置
        DeclareLaunchArgument(
            'playground', default_value='highway_lite'
        ),
        # SSC 配置文件参数: 默认保持原配置，实验时可传入其他 pb.txt 配置文件
        DeclareLaunchArgument(
            'ssc_config_path', default_value=default_ssc_config_path
        ),

        # ---------- 启动日志: 打印参数值便于确认配置 ----------
        LogInfo(msg=['arena_info_static_topic: ', LaunchConfiguration('arena_info_static_topic')]),
        LogInfo(msg=['arena_info_dynamic_topic: ', LaunchConfiguration('arena_info_dynamic_topic')]),
        LogInfo(msg=['ctrl_topic: ', LaunchConfiguration('ctrl_topic')]),
        LogInfo(msg=['playground: ', LaunchConfiguration('playground')]),
        LogInfo(msg=['ssc_config_path: ', LaunchConfiguration('ssc_config_path')]),
        LogInfo(msg="Launching node..."),

        # ---------- 定义节点 ----------
        Node(
            package='planning_integrated',         # 所属 ROS2 功能包
            executable='test_ssc_with_mpdm',       # 可执行文件名 (MPDM+SSC 集成节点)
            name='test_ssc_with_mpdm_0',           # 节点实例名称
            output='screen',                       # 将标准输出打印到终端
            parameters=[{                          # 传入节点的 ROS2 参数字典
                'ego_id': 0,                       # 自车ID: 默认为 0, 对应物理仿真器中的 agent_0
                'desired_vel': 60.0,               # 期望速度 (m/s): MPDM 版本默认更高, 适合高速场景
                'use_sim_state': True,             # 使用仿真器状态: True 表示从仿真器获取状态
                # 智能体配置文件路径: 定义车辆物理参数、传感器 FOV 等
                'agent_config_path': PathJoinSubstitution([
                    get_package_share_directory('playgrounds'),
                    LaunchConfiguration('playground'),
                    'agent_config.json'
                ]),
                # SSC 运动规划器配置文件路径: protobuf 格式, 定义轨迹优化参数
                # 注意: MPDM 版本不传入 bp_config_path, 因为 MPDM 行为策略内建在代码中
                'ssc_config_path': LaunchConfiguration('ssc_config_path')
            }],
            # remappings: 将节点内部使用的话题名重新映射到外部话题名
            remappings=[
                ('arena_info_static', LaunchConfiguration('arena_info_static_topic')),
                ('arena_info_dynamic', LaunchConfiguration('arena_info_dynamic_topic')),
                ('ctrl', LaunchConfiguration('ctrl_topic'))
            ]
        )
    ])
