#!/usr/bin/env python3
"""
@file phy_simulator_planning_launch.py
@brief 物理仿真器主启动脚本 —— 规划模式 (ROS2 Launch文件)

[架构定位]
本 launch 文件启动了 EPSILON 系统中物理仿真器 (phy_simulator) 的核心节点,
并顺带启动了 joystick 控制节点 (joy_ctrl), 为自动驾驶规划模块提供仿真环境。

[功能说明]
1. 声明启动参数 (arena_info_static_topic, arena_info_dynamic_topic, playground)
2. 通过 IncludeLaunchDescription 引入 joy_ctrl_launch.py 启动 joystick 节点
3. 启动 phy_simulator_planning_node 节点, 作为仿真的核心引擎

[仿真器工作原理]
phy_simulator 是 EPSILON 的物理仿真后端, 负责:
- 维护所有车辆的物理状态 (位置、速度、加速度、朝向)
- 接收各模块发布的 ctrl 话题 (vehicle_msgs::msg::ControlSignal), 通过车辆动力学模型更新状态
- 发布 arena_info_dynamic 话题 (所有车辆实时状态) 和 arena_info_static 话题 (地图/车道拓扑)

[参数说明]
- arena_info_static_topic: 静态竞技场信息输出话题名
- arena_info_dynamic_topic: 动态竞技场信息输出话题名
- playground: 运行场景名称 (如 highway_v1.0), 决定加载哪个场景的:
  - vehicle_set.json: 车辆集合定义 (车辆类型、初始位置、初始速度等)
  - obstacles_norm.json: 归一化后的障碍物/道路边界
  - lane_net_norm.json: 归一化后的车道网络

[话题映射]
- arena_info_static -> 参数指定的静态话题 (默认 /arena_info_static)
- arena_info_dynamic -> 参数指定的动态话题 (默认 /arena_info_dynamic)

[与 planning_integrated 的关系]
phy_simulator 和 planning_integrated 形成闭环:
  planning_integrated 发布 ctrl → phy_simulator 更新车辆状态 →
  phy_simulator 发布 arena_info → planning_integrated 重新规划 →
  循环往复形成闭环仿真
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    generate_launch_description —— ROS2 Launch 系统入口函数

    返回 LaunchDescription 对象, 其中包含:
    1. DeclareLaunchArgument: 声明 3 个可配置的启动参数
    2. IncludeLaunchDescription: 引入 joystick 控制节点 (joy_ctrl_launch.py)
    3. Node: 启动 phy_simulator_planning_node (物理仿真核心节点)
    4. LogInfo: 启动时打印所有关键参数值
    """

    # ---------- 声明启动参数 ----------
    # 静态竞技场信息话题: 发布地图、车道拓扑等不变信息
    arena_info_static_topic = DeclareLaunchArgument(
        'arena_info_static_topic', default_value='/arena_info_static'
    )
    # 动态竞技场信息话题: 发布所有车辆实时状态
    arena_info_dynamic_topic = DeclareLaunchArgument(
        'arena_info_dynamic_topic', default_value='/arena_info_dynamic'
    )
    # 场景名称参数: 决定加载哪个 playground 中的配置文件
    playground = DeclareLaunchArgument(
        'playground', default_value='highway_v1.0'
    )

    # ---------- 引入 Joystick 控制节点 ----------
    # joy_ctrl_launch.py 启动 ROS2 标准 joy_node, 连接物理手柄 (/dev/input/js0)
    joy_ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('phy_simulator'),
                'launch',
                'joy_ctrl_launch.py'
            ])
        ])
    )

    # ---------- 拼接配置文件路径 ----------
    # 车辆信息文件路径: 定义参与仿真车辆的类型、初始位置、初始速度等
    vehicle_info_path = PathJoinSubstitution([
        get_package_share_directory('playgrounds'),
        LaunchConfiguration('playground'),
        'vehicle_set.json'
    ])
    # 障碍物信息文件路径: 定义道路边界、静态障碍物等 (归一化坐标系)
    map_path = PathJoinSubstitution([
        get_package_share_directory('playgrounds'),
        LaunchConfiguration('playground'),
        'obstacles_norm.json'
    ])
    # 车道网络文件路径: 定义车道中心线拓扑 (归一化坐标系)
    lane_net_path = PathJoinSubstitution([
        get_package_share_directory('playgrounds'),
        LaunchConfiguration('playground'),
        'lane_net_norm.json'
    ])

    # ---------- 定义物理仿真器核心节点 ----------
    phy_simulator_planning_node = Node(
        package='phy_simulator',                     # 功能包名称
        executable='phy_simulator_planning_node',    # 可执行文件名 (物理仿真核心)
        name='phy_simulator_planning_node',          # 节点实例名称
        output='screen',                             # 将标准输出打印到终端
        parameters=[{                                # 传入节点的 ROS2 参数字典
            'vehicle_info_path': vehicle_info_path,   # 车辆配置路径
            'map_path': map_path,                     # 障碍物/地图路径
            'lane_net_path': lane_net_path            # 车道网络路径
        }],
        remappings=[
            # 话题重映射: 将内部话题名映射到外部话题名
            ('arena_info_static', LaunchConfiguration('arena_info_static_topic')),
            ('arena_info_dynamic', LaunchConfiguration('arena_info_dynamic_topic'))
        ]
    )

    # ---------- 组装 LaunchDescription ----------
    return LaunchDescription([
        arena_info_static_topic,
        arena_info_dynamic_topic,
        playground,
        joy_ctrl_launch,                              # 引入手柄控制节点
        # 启动日志
        LogInfo(msg=['arena_info_static_topic: ', LaunchConfiguration('arena_info_static_topic')]),
        LogInfo(msg=['arena_info_dynamic_topic: ', LaunchConfiguration('arena_info_dynamic_topic')]),
        LogInfo(msg=['playground: ', LaunchConfiguration('playground')]),
        LogInfo(msg=['vehicle_info_path: ', vehicle_info_path]),
        LogInfo(msg=['map_path: ', map_path]),
        LogInfo(msg=['lane_net_path: ', lane_net_path]),
        LogInfo(msg="Launching node..."),
        phy_simulator_planning_node
    ])
