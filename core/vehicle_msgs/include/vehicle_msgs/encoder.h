/**
 * @file encoder.h
 * @brief ROS 消息编码器 —— 将 common 命名空间的 C++ 数据结构编码为 vehicle_msgs ROS 消息
 *
 * @version 0.1
 * @date 2019
 *
 * 本模块实现了从 C++ 内部数据结构到 ROS 消息类型的单向转换。
 * 所有方法均为静态方法，无需实例化即可调用。
 *
 * 数据转换方向：C++ 内部数据结构 → ROS 消息
 *
 * 转换覆盖的数据类型：
 *   - 车辆状态：State（位置、速度、加速度、曲率、转向角）和 FreeState（自由状态）
 *   - 车辆定义：Vehicle、VehicleParam、VehicleSet
 *   - 车道网络：LaneRaw、LaneNet
 *   - 障碍物：CircleObstacle、PolygonObstacle、ObstacleSet
 *   - 竞技场信息：ArenaInfo、ArenaInfoStatic、ArenaInfoDynamic
 *   - 控制信号：ControlSignal（加速度、转向速率、开环标志）
 *   - 几何类型：Circle、Polygon
 *
 * 典型使用场景：
 *   - phy_simulator 发布场景数据：PhySimulation 内部数据 →
 *     Encoder::GetRosArenaInfoDynamicFromSimulatorData() → ROS 话题 /arena_info_dynamic
 *   - 发布控制信号：规划模块输出 common::VehicleControlSignal →
 *     Encoder::GetRosControlSignalFromControlSignal() → ROS 话题 /ctrl/agent_{id}
 *
 * 所有编码方法需要传入以下额外参数：
 *   - timestamp：ROS 时间戳（填充到消息头 header.stamp）
 *   - frame_id：参考坐标系名称（填充到消息头 header.frame_id，通常为 "map"）
 *
 * 与 Decoder（decoder.h）互为逆操作：
 *   Encoder: C++ 结构 → ROS 消息
 *   Decoder: ROS 消息 → C++ 结构
 */
#ifndef _VEHICLE_MSGS_INC_VEHICLE_MSGS_ENCODER_H__
#define _VEHICLE_MSGS_INC_VEHICLE_MSGS_ENCODER_H__

#include <algorithm>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"

#include "vehicle_msgs/msg/arena_info.hpp"
#include "vehicle_msgs/msg/arena_info_dynamic.hpp"
#include "vehicle_msgs/msg/arena_info_static.hpp"
#include "vehicle_msgs/msg/circle.hpp"
#include "vehicle_msgs/msg/circle_obstacle.hpp"
#include "vehicle_msgs/msg/control_signal.hpp"
#include "vehicle_msgs/msg/free_state.hpp"
#include "vehicle_msgs/msg/lane.hpp"
#include "vehicle_msgs/msg/lane_net.hpp"
#include "vehicle_msgs/msg/obstacle_set.hpp"
#include "vehicle_msgs/msg/occupancy_grid_float.hpp"
#include "vehicle_msgs/msg/occupancy_grid_u_int8.hpp"
#include "vehicle_msgs/msg/polygon_obstacle.hpp"
#include "vehicle_msgs/msg/state.hpp"
#include "vehicle_msgs/msg/vehicle.hpp"
#include "vehicle_msgs/msg/vehicle_param.hpp"
#include "vehicle_msgs/msg/vehicle_set.hpp"

#include "rclcpp/rclcpp.hpp"

namespace vehicle_msgs {

/// @brief ROS 消息编码器（静态工具类）
///
/// 提供一系列静态方法，将 EPSILON 内部通用数据结构编码为 ROS 格式的车辆消息，
/// 用于通过 ROS 话题发布给下游节点。
///
/// 命名约定：GetRos{TargetType}From{SourceType}()
///   例如：GetRosStateMsgFromState() —— 从 common::State 编码为 ros State 消息
///
/// 所有编码方法都会填充消息头的两个关键字段：
///   - header.stamp：ROS 时间戳（由调用方传入）
///   - header.frame_id：坐标系 ID（通常为 "map"）
class Encoder {
 public:
  /// @brief 将 common::FreeState 编码为 ROS FreeState 消息
  /// @param in_state 内部自由状态（x, y, vx, vy, ax, ay, angle）
  /// @param timestamp ROS 时间戳
  /// @param state [输出] ROS FreeState 消息
  /// @return kSuccess
  static ErrorType GetRosFreeStateMsgFromFreeState(
      const common::FreeState &in_state, const rclcpp::Time &timestamp,
      vehicle_msgs::msg::FreeState *state) {
    state->header.stamp = timestamp;
    state->header.frame_id = std::string("map");
    state->pos.x = in_state.position[0];
    state->pos.y = in_state.position[1];
    state->vel.x = in_state.velocity[0];
    state->vel.y = in_state.velocity[1];
    state->acc.x = in_state.acceleration[0];
    state->acc.y = in_state.acceleration[1];
    state->angle = in_state.angle;
    return kSuccess;
  }

  /// @brief 将 common::State 编码为 ROS State 消息
  /// @param in_state 内部车辆状态（位置、速度、加速度、曲率、转向角等）
  /// @param timestamp ROS 时间戳
  /// @param state [输出] ROS State 消息
  /// @return kSuccess
  static ErrorType GetRosStateMsgFromState(const common::State &in_state,
                                           const rclcpp::Time &timestamp,
                                           vehicle_msgs::msg::State *state) {
    state->header.stamp = timestamp;
    state->header.frame_id = std::string("map");
    state->vec_position.x = in_state.vec_position[0];
    state->vec_position.y = in_state.vec_position[1];
    state->angle = in_state.angle;
    state->curvature = in_state.curvature;
    state->velocity = in_state.velocity;
    state->acceleration = in_state.acceleration;
    state->steer = in_state.steer;
    return kSuccess;
  }

  /// @brief 将 common::VehicleSet 编码为 ROS VehicleSet 消息
  /// @param vehicle_set 内部车辆集合
  /// @param timestamp ROS 时间戳
  /// @param frame_id 坐标系 ID（如 "map"）
  /// @param msg [输出] ROS VehicleSet 消息
  /// @return kSuccess
  ///
  /// 遍历车辆集合中的所有车辆，逐辆编码并推入 msg->vehicles 数组。
  static ErrorType GetRosVehicleSetFromVehicleSet(
      const common::VehicleSet &vehicle_set, const rclcpp::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::msg::VehicleSet *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    for (const auto &p : vehicle_set.vehicles) {
      vehicle_msgs::msg::Vehicle vehicle_msg;
      GetRosVehicleFromVehicle(p.second, timestamp, frame_id, &vehicle_msg);
      msg->vehicles.push_back(vehicle_msg);
    }
    return kSuccess;
  }

  /// @brief 将 common::Vehicle 编码为 ROS Vehicle 消息
  /// @param vehicle 内部车辆对象
  /// @param timestamp ROS 时间戳
  /// @param frame_id 坐标系 ID
  /// @param msg [输出] ROS Vehicle 消息
  /// @return kSuccess
  ///
  /// 编码车辆的 ID、子类型、类型、物理参数和运动状态。
  static ErrorType GetRosVehicleFromVehicle(const common::Vehicle &vehicle,
                                            const rclcpp::Time &timestamp,
                                            const std::string &frame_id,
                                            vehicle_msgs::msg::Vehicle *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    msg->id.data = vehicle.id();
    msg->subclass.data = vehicle.subclass();
    msg->type.data = vehicle.type();
    vehicle_msgs::msg::VehicleParam param;
    GetRosVehicleParamFromVehicleParam(vehicle.param(), &param);
    msg->param = param;
    vehicle_msgs::msg::State state;
    GetRosStateMsgFromState(vehicle.state(), timestamp, &state);
    msg->state = state;
    return kSuccess;
  }

  /// @brief 将 common::VehicleParam 编码为 ROS VehicleParam 消息
  /// @param vehicle_param 内部车辆物理参数
  /// @param msg [输出] ROS VehicleParam 消息
  /// @return kSuccess
  ///
  /// 编码字段：车宽、车长、轴距、前后悬长度、
  /// 最大转向角、最大纵向/横向加速度、质心到后轴距离 d_cr。
  static ErrorType GetRosVehicleParamFromVehicleParam(
      const common::VehicleParam &vehicle_param,
      vehicle_msgs::msg::VehicleParam *msg) {
    msg->width = vehicle_param.width();
    msg->length = vehicle_param.length();
    msg->wheel_base = vehicle_param.wheel_base();
    msg->front_suspension = vehicle_param.front_suspension();
    msg->rear_suspension = vehicle_param.rear_suspension();
    msg->max_steering_angle = vehicle_param.max_steering_angle();
    msg->max_longitudinal_acc = vehicle_param.max_longitudinal_acc();
    msg->max_lateral_acc = vehicle_param.max_lateral_acc();
    msg->d_cr = vehicle_param.d_cr();
    return kSuccess;
  }

  /// @brief 将 common::LaneNet 编码为 ROS LaneNet 消息
  /// @param lane_net 内部车道网络
  /// @param timestamp ROS 时间戳
  /// @param frame_id 坐标系 ID
  /// @param msg [输出] ROS LaneNet 消息
  /// @return kSuccess
  ///
  /// 遍历所有车道并逐条编码，推入 msg->lanes 数组。
  static ErrorType GetRosLaneNetFromLaneNet(const common::LaneNet &lane_net,
                                            const rclcpp::Time &timestamp,
                                            const std::string &frame_id,
                                            vehicle_msgs::msg::LaneNet *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    for (const auto &lane_raw : lane_net.lane_set) {
      vehicle_msgs::msg::Lane lane;
      GetRosLaneFromLaneRaw(lane_raw.second, timestamp, frame_id, &lane);
      msg->lanes.push_back(lane);
    }
    return kSuccess;
  }

  /// @brief 将 common::LaneRaw 编码为 ROS Lane 消息
  /// @param lane 内部车道原始数据
  /// @param timestamp ROS 时间戳
  /// @param frame_id 坐标系 ID
  /// @param msg [输出] ROS Lane 消息
  /// @return kSuccess
  ///
  /// 编码完整的车道信息：ID、方向、拓扑连接关系、变道权限、
  /// 行为类型、长度、中心线采样点序列、起点和终点坐标。
  /// 注意：坐标的 z 分量统一设为 0.0（二维平面仿真）。
  static ErrorType GetRosLaneFromLaneRaw(const common::LaneRaw &lane,
                                         const rclcpp::Time &timestamp,
                                         const std::string &frame_id,
                                         vehicle_msgs::msg::Lane *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    msg->id = lane.id;
    msg->dir = lane.dir;
    msg->child_id = lane.child_id;
    msg->father_id = lane.father_id;
    msg->l_lane_id = lane.l_lane_id;
    msg->l_change_avbl = lane.l_change_avbl;
    msg->r_lane_id = lane.r_lane_id;
    msg->r_change_avbl = lane.r_change_avbl;
    msg->behavior = lane.behavior;
    msg->length = lane.length;
    // 起点坐标（z = 0.0）
    msg->start_point.x = lane.start_point(0);
    msg->start_point.y = lane.start_point(1);
    msg->start_point.z = 0.0;
    // 终点坐标（z = 0.0）
    msg->final_point.x = lane.final_point(0);
    msg->final_point.y = lane.final_point(1);
    msg->final_point.z = 0.0;
    // 车道中心线采样点序列
    for (const auto &pt : lane.lane_points) {
      geometry_msgs::msg::Point p;
      p.x = pt(0);
      p.y = pt(1);
      p.z = 0.0;
      msg->points.push_back(p);
    }
    return kSuccess;
  }

  /// @brief 将 common::ObstacleSet 编码为 ROS ObstacleSet 消息
  /// @param obstacle_set 内部障碍物集合
  /// @param timestamp ROS 时间戳
  /// @param frame_id 坐标系 ID
  /// @param msg [输出] ROS ObstacleSet 消息
  /// @return kSuccess
  ///
  /// 分别编码圆形障碍物（obs_circle）和多边形障碍物（obs_polygon），
  /// 推入 msg 各自的数组中。
  static ErrorType GetRosObstacleSetFromObstacleSet(
      const common::ObstacleSet &obstacle_set, const rclcpp::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::msg::ObstacleSet *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    // 编码圆形障碍物
    for (const auto obs : obstacle_set.obs_circle) {
      vehicle_msgs::msg::CircleObstacle obs_temp;
      GetRosCircleObstacleFromCircleObstacle(obs.second, timestamp, frame_id,
                                             &obs_temp);
      msg->obs_circle.push_back(obs_temp);
    }
    // 编码多边形障碍物
    for (const auto &obs : obstacle_set.obs_polygon) {
      vehicle_msgs::msg::PolygonObstacle obs_temp;
      GetRosPolygonObstacleFromPolygonObstacle(obs.second, timestamp, frame_id,
                                               &obs_temp);
      msg->obs_polygon.push_back(obs_temp);
    }
    return kSuccess;
  }

  /// @brief 将 common::CircleObstacle 编码为 ROS CircleObstacle 消息
  /// @param circle 内部圆形障碍物
  /// @param timestamp ROS 时间戳
  /// @param frame_id 坐标系 ID
  /// @param msg [输出] ROS CircleObstacle 消息
  /// @return kSuccess
  static ErrorType GetRosCircleObstacleFromCircleObstacle(
      const common::CircleObstacle &circle, const rclcpp::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::msg::CircleObstacle *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    msg->id = circle.id;
    GetRosCircleFromCircle(circle.circle, &msg->circle);
    return kSuccess;
  }

  /// @brief 将 common::PolygonObstacle 编码为 ROS PolygonObstacle 消息
  /// @param poly 内部多边形障碍物
  /// @param timestamp ROS 时间戳
  /// @param frame_id 坐标系 ID
  /// @param msg [输出] ROS PolygonObstacle 消息
  /// @return kSuccess
  static ErrorType GetRosPolygonObstacleFromPolygonObstacle(
      const common::PolygonObstacle &poly, const rclcpp::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::msg::PolygonObstacle *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    msg->id = poly.id;
    GetRosPolygonFromPolygon(poly.polygon, &msg->polygon);
    return kSuccess;
  }

  /// @brief 将 common::Circle 编码为 ROS Circle 消息
  /// @param circle 内部圆形（圆心坐标 + 半径）
  /// @param msg [输出] ROS Circle 消息
  /// @return kSuccess
  ///
  /// z 分量设为 0.0（二维平面仿真）。
  static ErrorType GetRosCircleFromCircle(const common::Circle &circle,
                                          vehicle_msgs::msg::Circle *msg) {
    msg->center.x = circle.center.x;
    msg->center.y = circle.center.y;
    msg->center.z = 0.0;
    msg->radius = circle.radius;
    return kSuccess;
  }

  /// @brief 将 common::Polygon 编码为 ROS Polygon 消息
  /// @param poly 内部多边形（顶点列表）
  /// @param msg [输出] ROS Polygon 消息
  /// @return kSuccess
  ///
  /// 每个顶点的 z 分量设为 0.0（二维平面仿真）。
  static ErrorType GetRosPolygonFromPolygon(const common::Polygon &poly,
                                            geometry_msgs::msg::Polygon *msg) {
    for (const auto p : poly.points) {
      geometry_msgs::msg::Point32 pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = 0.0;
      msg->points.push_back(pt);
    }
    return kSuccess;
  }

  /// @brief 将完整仿真数据编码为 ROS ArenaInfo 消息
  /// @param lane_net 车道网络
  /// @param vehicle_set 车辆集合
  /// @param obstacle_set 障碍物集合
  /// @param timestamp ROS 时间戳
  /// @param frame_id 坐标系 ID
  /// @param msg [输出] ROS ArenaInfo 消息
  /// @return kSuccess
  ///
  /// 这是最全面的编码入口，包含车道网络、车辆和障碍物全部三类数据。
  /// 消息发布到 /arena_info 话题。
  static ErrorType GetRosArenaInfoFromSimulatorData(
      const common::LaneNet &lane_net, const common::VehicleSet &vehicle_set,
      const common::ObstacleSet &obstacle_set, const rclcpp::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::msg::ArenaInfo *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    GetRosLaneNetFromLaneNet(lane_net, timestamp, frame_id, &msg->lane_net);
    GetRosVehicleSetFromVehicleSet(vehicle_set, timestamp, frame_id,
                                   &msg->vehicle_set);
    GetRosObstacleSetFromObstacleSet(obstacle_set, timestamp, frame_id,
                                     &msg->obstacle_set);
    return kSuccess;
  }

  /// @brief 将静态仿真数据编码为 ROS ArenaInfoStatic 消息
  /// @param lane_net 车道网络
  /// @param obstacle_set 障碍物集合
  /// @param timestamp ROS 时间戳
  /// @param frame_id 坐标系 ID
  /// @param msg [输出] ROS ArenaInfoStatic 消息
  /// @return kSuccess
  ///
  /// 仅编码静态数据（车道网络和障碍物），不包含车辆动态状态。
  /// 消息发布到 /arena_info_static 话题，建议以低频（~10 Hz）发送。
  static ErrorType GetRosArenaInfoStaticFromSimulatorData(
      const common::LaneNet &lane_net, const common::ObstacleSet &obstacle_set,
      const rclcpp::Time &timestamp, const std::string &frame_id,
      vehicle_msgs::msg::ArenaInfoStatic *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    GetRosLaneNetFromLaneNet(lane_net, timestamp, frame_id, &msg->lane_net);
    GetRosObstacleSetFromObstacleSet(obstacle_set, timestamp, frame_id,
                                     &msg->obstacle_set);
    return kSuccess;
  }

  /// @brief 将动态仿真数据编码为 ROS ArenaInfoDynamic 消息
  /// @param vehicle_set 车辆集合（所有车辆的实时状态）
  /// @param timestamp ROS 时间戳
  /// @param frame_id 坐标系 ID
  /// @param msg [输出] ROS ArenaInfoDynamic 消息
  /// @return kSuccess
  ///
  /// 仅编码动态数据（车辆实时状态），不包含静态地图信息。
  /// 消息发布到 /arena_info_dynamic 话题，建议以高频（~100 Hz）发送，
  /// 以确保规划模块获得足够实时的环境感知数据。
  static ErrorType GetRosArenaInfoDynamicFromSimulatorData(
      const common::VehicleSet &vehicle_set, const rclcpp::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::msg::ArenaInfoDynamic *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    GetRosVehicleSetFromVehicleSet(vehicle_set, timestamp, frame_id,
                                   &msg->vehicle_set);
    return kSuccess;
  }

  /// @brief 将 common::VehicleControlSignal 编码为 ROS ControlSignal 消息
  /// @param ctrl 内部控制信号
  /// @param timestamp ROS 时间戳
  /// @param frame_id 坐标系 ID
  /// @param msg [输出] ROS ControlSignal 消息
  /// @return kSuccess
  ///
  /// 编码的核心控制量：
  ///   - acc：纵向加速度
  ///   - steer_rate：转向角变化速率
  ///   - is_openloop：是否为开环控制标志
  ///   - state：开环模式下的目标状态（仅在 is_openloop == true 时有效）
  ///
  /// 此方法在规划模块输出控制指令时使用，消息通过 /ctrl/agent_{id} 话题
  /// 发送给 phy_simulator_planning_node 执行仿真。
  static ErrorType GetRosControlSignalFromControlSignal(
      const common::VehicleControlSignal &ctrl, const rclcpp::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::msg::ControlSignal *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    msg->acc = ctrl.acc;
    msg->steer_rate = ctrl.steer_rate;
    msg->is_openloop.data = ctrl.is_openloop;
    GetRosStateMsgFromState(ctrl.state, timestamp, &msg->state);
    return kSuccess;
  }
};  // class Encoder

}  // namespace vehicle_msgs

#endif  // _VEHICLE_MSGS_INC_VEHICLE_MSGS_ENCODER_H__
