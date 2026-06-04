#!/usr/bin/env python3
"""
@file joy_ctrl_launch.py
@brief Joystick/Gamepad 控制节点启动脚本 (ROS2 Launch文件)

[架构定位]
本 launch 文件是 EPSILON 系统中最底层的输入接口, 负责启动 ROS2 标准的 joy_node,
将物理游戏手柄 (Joystick/Gamepad) 的输入转换为 sensor_msgs/msg/Joy 消息,
通过 /joy 话题发布, 供 MPDM 行为规划器的 HMI 接口使用, 实现人在回路控制。

[功能说明]
1. 启动 ROS2 joy 包的 joy_node 节点
2. 配置设备路径为 /dev/input/js0 (Linux 下第一个 Joystick 设备)
3. 通过 /joy 话题发布手柄的轴和按钮状态

[设备要求]
- 需要一个物理 Joystick/Gamepad 连接到 Linux 系统
- 设备通常映射为 /dev/input/js0 (第一个手柄) 或 /dev/input/js1 (第二个手柄)
- 可通过 `ls /dev/input/js*` 查看可用设备

[与 terminal_server.py 的关系]
- joy_ctrl: 从物理手柄读取输入, 发布 /joy (sensor_msgs/msg/Joy)
- terminal_server.py: 从键盘读取输入, 也发布 /joy (sensor_msgs/msg/Joy)
  两者发布到同一话题, 实现了物理手柄和键盘的互替控制方案

[与 MPDM 行为规划器的交互]
Joy 消息的按钮索引语义:
  buttons[0]: 刹车
  buttons[1]: 右换道
  buttons[2]: 左换道
  buttons[3]: 加速
  buttons[4]: 切换左换道可行性
  buttons[5]: 切换右换道可行性
  buttons[6]: 切换自主模式

[被引入链]
phy_simulator_planning_launch.py → IncludeLaunchDescription → 本文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    generate_launch_description —— ROS2 Launch 系统入口函数

    返回 LaunchDescription 对象, 其中只包含一个 Node:
    - joy_node: ROS2 joy 包的节点, 将物理手柄数据转换为 Joy 消息
    """

    # ---------- 定义 Joystick 节点 ----------
    joy_node = Node(
        package='joy',             # ROS2 joy 功能包 (标准包)
        executable='joy_node',     # joy_node 可执行文件
        name='joy_node',           # 节点实例名称
        output='screen',           # 将标准输出打印到终端
        parameters=[{              # 节点参数
            'dev': '/dev/input/js0'  # Joystick 设备路径: Linux 下第一个手柄设备
        }]
    )

    return LaunchDescription([
        joy_node
    ])
