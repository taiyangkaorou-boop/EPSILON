#!/usr/bin/env python3
"""
@file proc_geojson.py
@brief GeoJSON 地图数据处理工具 —— 从 QGIS 导出数据生成仿真器输入文件

[架构定位]
本脚本是 EPSILON 仿真器地图预处理工具链的一部分, 用于将 QGIS 导出的 GeoJSON 格式
地图数据转换为物理仿真器 (phy_simulator) 可直接加载的归一化 JSON 格式。

[功能说明]
1. 读取三类 GeoJSON 文件: pt_feat (点特征), lane_net (车道网络), obstacles (障碍物)
2. 以 origin 点为原点, 将所有坐标归一化 (平移使 origin 在 (0,0))
3. 将归一化后的数据保存为 *_norm.json 文件
4. 使用 Matplotlib 绘制地图可视化 (车道拓扑 + 障碍物轮廓)

[输入文件] (GeoJSON 格式, 由 QGIS 手动标注导出)
- {data_folder}/pt_feat.geojson:     点特征 (包含 origin 原点定义)
- {data_folder}/lane_net.geojson:    车道网络拓扑 (车道中心线)
- {data_folder}/obstacles.geojson:   障碍物边界 (多边形)

[输出文件] (归一化 JSON 格式, 供 phy_simulator 加载)
- {data_folder}/lane_net_norm.json:  归一化车道网络
- {data_folder}/obstacles_norm.json: 归一化障碍物

[归一化原理]
所有坐标减去 origin 点的坐标, 使地图的原点始终在 (0, 0)。
这样做的好处:
- 避免大坐标值带来的浮点精度损失
- 简化各路模块的坐标计算
- 使不同场景的地图可以有统一的空间参考

[使用方法]
1. 在 QGIS 中标注车道网络和障碍物, 导出为 GeoJSON
2. 将 GeoJSON 文件放入 data_folder 指定的目录
3. 运行本脚本: python proc_geojson.py
4. 将生成的 *_norm.json 文件放入对应 playground 目录
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

import math
from pprint import pprint

# data_folder: GeoJSON 数据源文件夹路径 (相对路径, 相对于脚本所在目录)
# 可通过修改此变量指定不同的场景数据源
data_folder = '../../../toolchain/qgis_projects/highway/geojson/'

print(data_folder)

# ---------- 读取点特征 GeoJSON ----------
# pt_feat.geojson: 包含关键点特征, 其中 "origin" 点是坐标归一化的参考原点
pt_feat_name = data_folder + "pt_feat" + ".geojson"
pt_feat_json_fs = open(pt_feat_name).read()
pt_feat_json = json.loads(pt_feat_json_fs)

# ---------- 读取车道网络 GeoJSON ----------
# lane_net.geojson: 包含车道中心线的多段线 (LineString) 几何数据
lane_net_name = data_folder + "lane_net" + ".geojson"
lane_net_json_fs = open(lane_net_name).read()
lane_net_json = json.loads(lane_net_json_fs)

# ---------- 读取障碍物 GeoJSON ----------
# obstacles.geojson: 包含障碍物/道路边界的多边形 (Polygon) 几何数据
obstacles_name = data_folder + "obstacles" + ".geojson"
obstacles_json_fs = open(obstacles_name).read()
obstacles_json = json.loads(obstacles_json_fs)

# ---------- 提取 origin 点 ----------
# 在 pt_feat 的所有特征中查找名为 "origin" 的点特征,
# 将其坐标作为归一化的参考原点
for f in pt_feat_json["features"]:
  if f["properties"]["name"] == "origin":
    origin = np.array(f["geometry"]["coordinates"])

# ---------- 坐标归一化: 车道网络 ----------
# 将每条车道中心线的所有点坐标减去 origin, 使地图以 origin 为中心
for f in lane_net_json["features"]:
  for pt in f["geometry"]["coordinates"][0]:
    pt[0] = pt[0] - origin[0]  # x 坐标归一化
    pt[1] = pt[1] - origin[1]  # y 坐标归一化

# ---------- 坐标归一化: 障碍物 ----------
# 将每个障碍物多边形的所有顶点坐标减去 origin
for f in obstacles_json["features"]:
  for pt in f["geometry"]["coordinates"][0][0]:
    pt[0] = pt[0] - origin[0]  # x 坐标归一化
    pt[1] = pt[1] - origin[1]  # y 坐标归一化

# ---------- 保存归一化数据 ----------
# 将归一化后的车道网络写入 lane_net_norm.json
lane_net_norm_name = data_folder + "lane_net_norm" + ".json"
lane_net_norm_json_fs = open(lane_net_norm_name,'w')
json.dump(lane_net_json, lane_net_norm_json_fs)

# 将归一化后的障碍物写入 obstacles_norm.json
obstacles_norm_name = data_folder + "obstacles_norm" + ".json"
obstacles_norm_json_fs = open(obstacles_norm_name,'w')
json.dump(obstacles_json, obstacles_norm_json_fs)

# ---------- 可视化: 绘制地图数据 ----------
x_set = []  # 每条车道中心线的 x 坐标数组列表
y_set = []  # 每条车道中心线的 y 坐标数组列表

# 提取车道中心线坐标用于绘图
for f in lane_net_json["features"]:
  coord = np.array(f["geometry"]["coordinates"][0])
  x = coord[:,0]
  y = coord[:,1]
  x_set.append(x)
  y_set.append(y)

# 创建 Matplotlib 图形和坐标轴
fig, ax = plt.subplots()

# 绘制所有车道中心线 (带圆形标记)
for i in range(len(x_set)):
  ax.plot(x_set[i], y_set[i], marker='o')

# 收集所有障碍物多边形
patches = []
for f in obstacles_json["features"]:
  coord = np.array(f["geometry"]["coordinates"][0][0])
  polygon = Polygon(np.array(coord), True)  # True 表示闭合多边形
  patches.append(polygon)

# 创建 PatchCollection 并添加到坐标轴
p = PatchCollection(patches, cmap=matplotlib.cm.jet, alpha=0.4)

# 用随机颜色给不同障碍物着色, 便于区分
colors = 100 * np.random.rand(len(patches))
p.set_array(np.array(colors))
ax.add_collection(p)

# 设置等比例坐标轴并显示
plt.axis('equal')
plt.show()
