#!/usr/bin/env python3
"""
@file terminal_server.py
@brief 基于 Pygame 的交互式终端服务器 —— 人在回路 (Human-in-the-Loop) 远程控制

[架构定位]
本脚本是 EPSILON 自动驾驶系统中的人机交互 (HMI) 终端, 通过 Pygame 图形界面
提供车辆状态的 2D 鸟瞰视图和 W/A/S/D 键盘远程控制功能。

[功能说明]
1. 2D 可视化: 在地图画布上实时渲染所有车辆的位置、朝向、ID 标签以及车道拓扑
2. 鼠标点选: 点击某个车辆即可将其选为当前遥控目标
3. 键盘控制: W(加速) / S(刹车) / A(左换道) / D(右换道)
4. 高级控制: Q(左换道可行性切换) / E(右换道可行性切换) / R(自主模式切换)
5. 方向盘可视化: 从 ControlSignal 消息中提取曲率信息, 渲染方向盘转动动画
6. 速度/加速度 HUD: 在屏幕上实时显示被控车辆的速度和加速度

[数据流]
   键盘/WASD 输入 → Joy 消息 (sensor_msgs/msg/Joy) → 行为规划器 HMI 接口
   arena_info_dynamic → 车辆位置更新 → Pygame 渲染
   arena_info_static → 车道网络数据 → 车道线绘制
   /ctrl/agent_0 → 控制信号 → 方向盘转角和 HUD 速度显示

[消息格式: Joy 按钮映射]
   buttons[0] = W: 加速
   buttons[1] = D: 右换道
   buttons[2] = A: 左换道
   buttons[3] = S: 刹车
   buttons[4] = Q: 切换左换道可行性
   buttons[5] = E: 切换右换道可行性
   buttons[6] = R: 切换自主模式
   Joy.header.frame_id = 被控车辆的 agent_id 字符串

[与行为规划器的交互]
   Joy 消息被 MPDM 行为规划器的 HMI 接口接收, 用于触发以下语义级指令:
   - 加速: 增加期望速度
   - 刹车: 降低期望速度或紧急制动
   - 换道: 触发左/右换道策略
   - 可行性切换: 标记某个换道方向为不可行
   - 自主模式切换: 在人工控制和 AI 自主控制间切换
"""

# sys
import time
import sys
import shutil
import random
from math import *
from functools import reduce

# pygame: 2D 图形渲染库, 用于绘制车辆位置、车道线和 HUD 信息
import pygame as pg
from pygame.locals import *
from pygame.math import Vector2

# ROS2: rclpy 是 Python 版本的 ROS2 客户端库
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

# msg: ROS2 消息类型定义
from geometry_msgs.msg import Twist        # 通用速度指令 (未在此文件直接使用)
from sensor_msgs.msg import Joy            # Joystick/Joypad 消息, 用于 HMI 控制
from vehicle_msgs.msg import ArenaInfoDynamic  # 动态竞技场信息 (车辆实时位置/速度)
from vehicle_msgs.msg import ArenaInfoStatic   # 静态竞技场信息 (车道拓扑/地图)
from vehicle_msgs.msg import State             # 车辆状态 (位置/速度/角度/曲率)
from vehicle_msgs.msg import ControlSignal     # 控制信号 (方向盘转角/油门/刹车)

# ---------- 全局变量 ----------
ego_id = 0                       # 自车 ID, 通常为 0
agent_id = 0                     # 当前被遥控的智能体 ID (鼠标点击选择)
width = 800                      # Pygame 窗口宽度 (像素)
height = 600                     # Pygame 窗口高度 (像素)
scale = 4.0                      # 世界坐标到像素坐标的缩放比例 (像素/米)
screen = pg.display.set_mode((width, height))  # Pygame 绘图表面
all_sprites = pg.sprite.Group()  # Pygame 精灵组: 统一管理所有可绘制对象
vehicles = {}                    # 车辆字典: {vehicle_id: State}, 存储所有车辆的当前状态
recorded_ids = []                # 已记录的车辆 ID 列表, 用于创建 Vehicle 精灵
lane_pts = []                    # 车道点列表: [[(u,v), ...], ...], 已变换到图像坐标
center_3dof = (0.0, 0.0, 0.0)   # 画面中心点的世界坐标 (x, y, angle), 随自车移动
state_seq = []                   # 控制信号历史序列, 用于计算方向盘转角和加速度
has_arena_info_dynamic = False   # 标志: 是否已收到过动态竞技场信息

class TerminalServerNode(Node):
    """
    TerminalServerNode —— ROS2 节点类, 负责订阅车辆状态话题和发布 Joy 控制指令

    该节点运行在 rclpy.spin 中, 通过定时器以 50Hz 频率:
    1. 处理键盘事件 (handle_keyboard_event)
    2. 更新 Pygame 可视化 (update_visualization)
    """

    def __init__(self):
        """
        构造函数: 初始化 ROS2 发布者、订阅者和定时器

        发布者:
          - joy_pub: 发布 Joy 消息到 /joy 话题, 用于 MPDM 行为规划器的 HMI 接口

        订阅者:
          - /arena_info_dynamic: 动态竞技场信息 → process_arena_info_dynamic()
          - /arena_info_static: 静态竞技场信息 → process_arena_info_static()
          - /ctrl/agent_0: 自车控制信号 → process_control_signal()
        """
        super().__init__('key2joy')
        # 创建 Joy 消息发布者: 队列深度 10, 发布到 /joy 话题
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        # 订阅动态竞技场信息: 更新所有车辆的位置和状态
        self.create_subscription(ArenaInfoDynamic, "/arena_info_dynamic", self.process_arena_info_dynamic, 10)
        # 订阅静态竞技场信息: 更新车道网络拓扑
        self.create_subscription(ArenaInfoStatic, "/arena_info_static", self.process_arena_info_static, 10)
        # 订阅自车控制信号: 用于方向盘动画和速度/加速度 HUD
        self.create_subscription(ControlSignal, "/ctrl/agent_0", self.process_control_signal, 10)
        # 创建定时器: 50Hz, 定期调用 timer_callback
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)  # 50 Hz

    def timer_callback(self):
        """
        定时器回调: 每 20ms 调用一次 (50Hz)

        执行流程:
          1. handle_keyboard_event: 检查键盘事件, 有按键则发布 Joy 消息
          2. update_visualization: 刷新 Pygame 画布, 更新所有图形元素
          3. pg.display.update(): 将后台缓冲区内容显示到屏幕
        """
        handle_keyboard_event(self.joy_pub)
        update_visualization()
        pg.display.update()

    def process_arena_info_dynamic(self, data):
        """
        处理动态竞技场信息回调: 更新所有车辆的状态

        [参数] data: ArenaInfoDynamic 消息, 包含当前时刻所有车辆的位置/速度/朝向
        [功能]
          1. 遍历所有车辆, 更新 vehicles 字典
          2. 为新出现的车辆创建 Vehicle 精灵 (Pygame 图形对象)
          3. 将画面中心 (center_3dof) 设置为自车当前坐标, 实现"跟随视角"
        """
        global center_3dof, has_arena_info_dynamic
        for v in data.vehicle_set.vehicles:
            # 更新或创建车辆状态记录
            vehicles[v.id.data] = v.state
            if v.id.data not in recorded_ids:
                # 新车辆: 创建 Pygame 精灵后加入精灵组
                recorded_ids.append(v.id.data)
                screen_rect = screen.get_rect()
                all_sprites.add(Vehicle(screen_rect, v.id.data, (v.state.vec_position.x,
                                                                 v.state.vec_position.y, v.state.angle)))
        # 更新画面中心为自车当前坐标 (跟随自车视角)
        center_3dof = (vehicles[ego_id].vec_position.x,
                       vehicles[ego_id].vec_position.y, vehicles[ego_id].angle)
        has_arena_info_dynamic = True

    def process_arena_info_static(self, data):
        """
        处理静态竞技场信息回调: 提取视口范围内的车道点

        [参数] data: ArenaInfoStatic 消息, 包含车道网络拓扑 (车道中心线)
        [功能] 遍历所有车道, 提取视口可见范围 (150m) 内的车道点,
               并变换到图像坐标后存入 lane_pts 列表

        [性能优化] visible_range = 150m: 只绘制可见范围内的车道,
                   避免绘制远处不可见的车道导致性能下降
        """
        global lane_pts
        if has_arena_info_dynamic:
            visible_range = 150.0           # 可见范围 (米)
            del lane_pts[:]                  # 清空旧的车道点, 重新计算
            for lane in data.lane_net.lanes:
                points = []
                for pt in lane.points:
                    # 只选取视口可见范围内的车道点
                    if abs(pt.x - center_3dof[0]) < visible_range and abs(pt.y - center_3dof[1]) < visible_range:
                        # 转换到图像坐标 (适配 Pygame 画布)
                        points.append(project_world_to_image((pt.x, pt.y, 0.0)))
                if len(points) > 2:
                    lane_pts.append(points)

    def process_control_signal(self, data):
        """
        处理控制信号回调: 维护控制信号的历史序列

        [参数] data: ControlSignal 消息, 包含当前时刻的车辆状态 (含曲率)
        [功能] 将最新状态追加到 state_seq, 保持最近 10 个状态用于:
               1. 计算平均方向盘转角 (用于方向盘动画)
               2. 计算当前加速度 (用于 HUD 显示)
        """
        global state_seq
        state_seq.append(data.state)
        if len(state_seq) > 10:
            state_seq.pop(0)  # 维护滑动窗口, 只保留最近 10 条

def project_world_to_image(point_3dof):
    """
    将世界坐标 (x, y, angle) 转换为 Pygame 画布上的像素坐标 (u, v)

    [变换原理]
    1. 平移到以 center_3dof 为原点的相对坐标系
    2. 绕 Z 轴旋转角度 center_3dof[2], 使自车朝向始终"向上"
    3. 缩放: 乘以 scale 因子 (像素/米)
    4. 偏移: 使 (0,0) 映射到画布中心 (width/2, height/2)

    [注意] v 坐标取负是因为 Pygame 的 y 轴向下为正, 而世界坐标的 y 轴向上为正

    [参数] point_3dof: (x_world, y_world, angle) 世界坐标
    [返回] (u, v): Pygame 画布像素坐标
    """
    x = point_3dof[0] - center_3dof[0]
    y = point_3dof[1] - center_3dof[1]
    angle = center_3dof[2]
    u = width / 2 + scale * (x * sin(angle) - y * cos(angle))
    v = height / 2 - scale * (x * cos(angle) + y * sin(angle))
    return (u, v)

class Wheel(pg.sprite.Sprite):
    """
    Wheel —— 方向盘可视化精灵

    在画布左上角 (100, 100) 显示一个方向盘图标,
    根据 control signal 中的曲率信息实时旋转方向盘。
    方向盘角度对应自车的实际转向角, 通过曲率 (curvature)
    乘以轴距 (2.85m) 反算出前轮偏角。
    """

    def __init__(self, screen_rect):
        """
        构造函数: 加载方向盘图片并缩放到合适大小

        [参数] screen_rect: Pygame 画布的矩形区域 (用于定位)
        """
        pg.sprite.Sprite.__init__(self)
        self.original_image = pg.image.load("steer_wheel.png").convert_alpha()
        self.original_image = pg.transform.rotozoom(self.original_image, 0.0, 0.2)
        self.image = self.original_image
        self.rect = self.image.get_rect()
        self.rect.center = (100, 100)  # 方向盘图标固定在画布左上角

    def update(self):
        """
        更新方向盘的旋转角度

        从 control signal 序列中计算平均曲率, 反算前轮偏角,
        然后将偏角从弧度转换为度, 最后旋转方向盘精灵。
        """
        angle, acc = calc_current_steer_acc()
        angle = angle * 180 / pi    # 弧度转度
        self.image = pg.transform.rotate(self.original_image, angle)
        x, y = self.rect.center
        self.rect = self.image.get_rect()  # 替换旧的 rect
        self.rect.center = (x, y)          # 保持中心不变

class Vehicle(pg.sprite.Sprite):
    """
    Vehicle —— 车辆可视化精灵

    每个 Vehicle 精灵对应一辆仿真中的车辆, 在画布上以圆形绘制,
    自车用浅绿色 (darkolivegreen1) 区分, 其他车辆用浅蓝色 (dodgerblue1)。
    """

    def __init__(self, screen_rect, id, state_3dof):
        """
        构造函数: 创建圆形车辆标记

        [参数]
          - screen_rect: Pygame 画布的矩形区域
          - id: 车辆的唯一标识符
          - state_3dof: 车辆的初始世界坐标 (x, y, angle)
        """
        pg.sprite.Sprite.__init__(self)
        self.id = id
        self.state_3dof = state_3dof
        self.radius = 10  # 车辆圆形标记的半径 (像素)
        # 创建带透明通道的 Surface 作为车辆图像
        self.image = pg.Surface((self.radius * 2, self.radius * 2), pg.SRCALPHA)
        # 自车和周围车辆用不同颜色区分
        if id == ego_id:
            pg.draw.circle(
                self.image, pg.Color('darkolivegreen1'), (self.radius, self.radius), self.radius)
        else:
            pg.draw.circle(
                self.image, pg.Color('dodgerblue1'), (self.radius, self.radius), self.radius)

        # 设置初始位置
        self.rect = self.image.get_rect(center=project_world_to_image(state_3dof))
        self.screen_rect = screen_rect

    def update(self):
        """
        更新车辆在画布上的位置

        从 vehicles 字典中获取该车辆的最新世界坐标,
        通过 project_world_to_image 转换为画布坐标后更新 rect.center。
        """
        latest_state_3dof = (vehicles[self.id].vec_position.x,
                             vehicles[self.id].vec_position.y, vehicles[self.id].angle)
        self.rect.center = project_world_to_image(latest_state_3dof)

def calc_current_steer_acc():
    """
    从控制信号序列中计算当前方向盘转角和加速度

    [计算逻辑]
    1. 曲率 (curvature) → 前轮偏角: steer = atan(curvature * 2.85)
       其中 2.85 是车辆轴距 (m)
    2. 对历史偏角取平均: 平滑显示, 避免方向盘抖动
    3. 乘缩放因子 1.25 * (360/45): 将实际偏角映射到方向盘显示偏角

    [返回] (rotated_angle, acceleration):
      - rotated_angle: 方向盘显示角度 (度)
      - acceleration: 最新控制信号中的加速度 (m/s^2)
    """
    if len(state_seq) < 2:
        return 0.0, 0.0
    steer_list = []
    for i in range(len(state_seq) - 1):
        state1 = state_seq[i]
        # steer = atan(curvature * wheelbase): 由曲率和轴距反算前轮偏角
        steer = atan(state1.curvature * 2.85)
        steer_list.append(steer)
    # reduce 求和后求平均: 平滑方向盘转动效果
    ave_steer = reduce(lambda x, y: x + y, steer_list) / len(steer_list)
    # 映射到显示角度: 实际偏角 * 放大因子 * 度转换
    rotated_angle = ave_steer * 1.25 * (360.0 / 45.0)
    return rotated_angle, state_seq[-1].acceleration

def plot_lanes_on_screen():
    """
    在画布上绘制车道拓扑

    遍历 lane_pts 列表, 用亮粉色 (deeppink) 绘制每条车道的中心线。
    lane_pts 中的坐标已经是经过 project_world_to_image 变换后的画布坐标。
    """
    lanepts_plot = list(lane_pts)
    for i in range(len(lanepts_plot)):
        pg.draw.lines(screen, pg.Color('deeppink'), False, lanepts_plot[i])

def plot_speed_on_screen():
    """
    在画布上显示速度和加速度 HUD

    从 state_seq 的最后一条记录中提取:
    - 速度: 转换为 km/h (乘以 3.6)
    - 加速度: 保持 m/s^2

    在画布左上角 (110, 180) 和 (110, 200) 显示文本。
    """
    speed = 0.0
    acc = 0.0
    if len(state_seq) > 1:
        speed = state_seq[-1].velocity * 3.6      # m/s -> km/h
        acc = state_seq[-1].acceleration
    font_obj = pg.font.Font('freesansbold.ttf', 20)
    # 绘制速度文本: "vel: XX.XX km/h"
    text_surface_obj = font_obj.render(
        'vel: {:.2f} km/h '.format(speed), True, (0, 0, 0))
    text_rect_obj = text_surface_obj.get_rect()
    text_rect_obj.center = (110, 180)
    screen.blit(text_surface_obj, text_rect_obj)
    # 绘制加速度文本: "acc: XX.XX m/s^2"
    text_surface_obj = font_obj.render(
        'acc: {:.2f} m/s^2'.format(acc), True, (0, 0, 0))
    text_rect_obj = text_surface_obj.get_rect()
    text_rect_obj.center = (110, 200)
    screen.blit(text_surface_obj, text_rect_obj)

def plot_ids_on_screen():
    """
    在画布上绘制车辆 ID 标签

    遍历所有已记录的车辆, 在每辆车左侧 20 像素处显示其 ID 编号,
    使用绿色 (0, 255, 0) 文本渲染。
    """
    for idx in recorded_ids:
        state_3dof = (vehicles[idx].vec_position.x,
                      vehicles[idx].vec_position.y, vehicles[idx].angle)
        font_obj = pg.font.Font('freesansbold.ttf', 16)
        text_surface_obj = font_obj.render(
            '{}'.format(idx), True, (0, 255, 0))
        text_rect_obj = text_surface_obj.get_rect()
        u, v = project_world_to_image(state_3dof)
        text_rect_obj.center = (u - 20, v)
        screen.blit(text_surface_obj, text_rect_obj)

def plot_orientations_on_screen():
    """
    在画布上绘制车辆朝向线

    遍历所有已记录的车辆, 在每辆车位置画一条黑色线段表示车辆朝向。
    线段方向 = 车辆世界角度 - 自车世界角度 (相对角度)。
    """
    for idx in recorded_ids:
        state_3dof = (vehicles[idx].vec_position.x,
                      vehicles[idx].vec_position.y, vehicles[idx].angle)
        u, v = project_world_to_image(state_3dof)
        angle_diff = vehicles[idx].angle - center_3dof[2]  # 相对自车朝向的角度差
        pt1 = (u, v)
        pt2 = (u - 10 * sin(angle_diff), v - 10 * cos(angle_diff))
        pg.draw.line(screen, pg.Color('black'), pt1, pt2, 3)

def plot_selected_rect_on_screen():
    """
    在画布上高亮当前选中的遥控目标车辆

    在 agent_id 对应的车辆周围绘制一个浅蓝色 (aquamarine3) 矩形框,
    宽高各 20 像素, 线宽 3 像素, 用于提示当前遥控目标。
    """
    if agent_id in recorded_ids:
        agent_state = (vehicles[agent_id].vec_position.x,
                       vehicles[agent_id].vec_position.y, vehicles[agent_id].angle)
        u, v = project_world_to_image(agent_state)
        pg.draw.rect(screen, pg.Color('aquamarine3'), (u - 10, v - 10, 20, 20), 3)

def update_visualization():
    """
    综合可视化更新函数: 按顺序重绘所有图形元素

    [绘制顺序]
    1. all_sprites.update():  更新所有精灵的位置和状态
    2. screen.fill():          用背景色填充, 清除上帧内容
    3. all_sprites.draw():     绘制所有精灵 (车辆圆形 + 方向盘)
    4. plot_lanes_on_screen:   绘制车道拓扑
    5. plot_ids_on_screen:     绘制车辆 ID 标签
    6. plot_selected_rect:     高亮选中车辆
    7. plot_speed_on_screen:   显示速度/加速度 HUD
    8. plot_orientations:      绘制车辆朝向线
    """
    if has_arena_info_dynamic:
        all_sprites.update()
        screen.fill(pg.Color('cornsilk2'))  # 米色背景
        all_sprites.draw(screen)
        plot_lanes_on_screen()
        plot_ids_on_screen()
        plot_selected_rect_on_screen()
        plot_speed_on_screen()
        plot_orientations_on_screen()

def init_joy(frame_id):
    """
    初始化一个 Joy 消息对象

    [参数] frame_id: 字符串, 被控车辆的 ID (如 "0", "3"), 用于行为规划器识别目标车辆
    [返回] 已初始化的 Joy 消息, 包含 8 个轴 (全为 0.0) 和 11 个按钮 (全为 0)

    [注意] Joy 消息在 ROS2 中被用作 HMI 接口协议, 并非真正的物理手柄信号,
          而是通过按钮索引编码不同的语义指令。
    """
    joy = Joy()
    joy.header.frame_id = frame_id
    joy.header.stamp = Clock().now().to_msg()
    for i in range(8):
        joy.axes.append(0.0)
    for i in range(11):
        joy.buttons.append(0)
    return joy

def handle_keyboard_event(joy_pub):
    """
    处理键盘和鼠标事件 —— 人在回路控制的核心入口

    [鼠标事件]
    - MOUSEBUTTONUP: 点击车辆圆形标记, 将其选为当前遥控目标 (更新 agent_id)

    [键盘事件映射 (Joy 按钮)]
    - W (buttons[3]): 加速 —— 增加期望速度
    - S (buttons[0]): 刹车 —— 降低期望速度或紧急制动
    - A (buttons[2]): 左换道 —— 向左变道
    - D (buttons[1]): 右换道 —— 向右变道
    - Q (buttons[4]): 切换左换道可行性 —— 标记/取消左侧车道为不可用
    - E (buttons[5]): 切换右换道可行性 —— 标记/取消右侧车道为不可用
    - R (buttons[6]): 切换自主模式 —— 在人工控制和 AI 自主控制之间切换

    [数据流] 键盘事件 → Joy 消息 → /joy 话题 → MPDM 行为规划器 HMI 接口
    """
    global agent_id
    for event in pg.event.get():
        # 鼠标点击: 选择遥控目标
        if event.type == pg.MOUSEBUTTONUP:
            pos = pg.mouse.get_pos()
            clicked_sprites = [s for s in all_sprites if s.rect.collidepoint(pos)]
            if len(clicked_sprites) > 0:
                if hasattr(clicked_sprites[0], 'id'):
                    agent_id = clicked_sprites[0].id
                    print('update agent id to ', agent_id)

        # 键盘按下: 发送 Joy 指令
        if event.type == KEYDOWN:
            joy = init_joy("{}".format(agent_id))  # 将当前目标 ID 写入 frame_id
            if event.key == pg.K_w:
                msg = 'Agent {}: Speed up'.format(agent_id)
                print(msg)
                joy.buttons[3] = 1                 # 按钮 3: 加速
                joy_pub.publish(joy)
            elif event.key == pg.K_s:
                msg = 'Agent {}: Brake'.format(agent_id)
                print(msg)
                joy.buttons[0] = 1                 # 按钮 0: 刹车
                joy_pub.publish(joy)
            elif event.key == pg.K_a:
                msg = 'Agent {}: Lane change left'.format(agent_id)
                print(msg)
                joy.buttons[2] = 1                 # 按钮 2: 左换道
                joy_pub.publish(joy)
            elif event.key == pg.K_d:
                msg = 'Agent {}: Lane change right'.format(agent_id)
                print(msg)
                joy.buttons[1] = 1                 # 按钮 1: 右换道
                joy_pub.publish(joy)
            elif event.key == pg.K_q:
                msg = 'Agent {}: Toggle left lc feasible state'.format(agent_id)
                print(msg)
                joy.buttons[4] = 1                 # 按钮 4: 切换左换道可行性
                joy_pub.publish(joy)
            elif event.key == pg.K_e:
                msg = 'Agent {}: Toggle right lc feasible state'.format(agent_id)
                print(msg)
                joy.buttons[5] = 1                 # 按钮 5: 切换右换道可行性
                joy_pub.publish(joy)
            elif event.key == pg.K_r:
                msg = 'Agent {}: Toggle autonomous mode'.format(agent_id)
                print(msg)
                joy.buttons[6] = 1                 # 按钮 6: 切换自主模式
                joy_pub.publish(joy)

def main(args=None):
    """
    主函数: 初始化 ROS2 和 Pygame, 启动终端服务器

    [执行流程]
    1. rclpy.init():   初始化 ROS2 客户端库
    2. pg.init():      初始化 Pygame 显示子系统
    3. 创建 TerminalServerNode (ROS2 节点)
    4. rclpy.spin(node): 进入 ROS2 事件循环, 通过定时器驱动 Pygame 更新
    5. 清理: pg.quit() + rclpy.shutdown()
    """
    rclpy.init(args=args)
    pg.init()
    screen.fill(pg.Color('cornsilk3'))
    screen_rect = screen.get_rect()
    all_sprites.add(Wheel(screen_rect))  # 添加方向盘精灵
    all_sprites.draw(screen)
    pg.display.set_caption('Ultimate Vehicle Planning')
    pg.display.update()

    node = TerminalServerNode()

    print('Terminal server initialized.')
    rclpy.spin(node)  # 阻塞运行, 直到节点被关闭 (Ctrl+C)

    pg.quit()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
